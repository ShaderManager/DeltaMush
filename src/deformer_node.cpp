#include <maya/MFnNumericAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MItGeometry.h>
#include <maya/MFnMesh.h>
#include <maya/MPointArray.h>
#include <maya/MMatrix.h>
#include <maya/MFnMeshData.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnMessageAttribute.h>

#include <maya/MFnVectorArrayData.h>

#include <vector>
#include <tbb/parallel_for.h>

#include "deformer_node.hpp"

MString DeltaMushDeformer::type_name("deltaMush");
MTypeId DeltaMushDeformer::type_id(DeltaMushDeformer::TypeId_Prefix, DeltaMushDeformer::TypeId_Value);

MObject DeltaMushDeformer::attr_smooth_iterations;
MObject DeltaMushDeformer::attr_delta_offsets;
MObject DeltaMushDeformer::attr_deformer_mode;

MStatus DeltaMushDeformer::initializer()
{
	MStatus status;
	MFnNumericAttribute num_attr;
	MFnTypedAttribute typed_attr;
	MFnEnumAttribute enum_attr;

	attr_smooth_iterations = num_attr.create("smoothIterations", "smit", MFnNumericData::kLong, 3, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	num_attr.setMin(0);

	attr_delta_offsets = typed_attr.create("deltaOffsets", "dlta", MFnData::kVectorArray, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	typed_attr.setConnectable(false);
	typed_attr.setKeyable(false);
	typed_attr.setHidden(true);

	attr_deformer_mode = enum_attr.create("mode", "mode", 0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	enum_attr.addField("Delta Mush", 0);
	enum_attr.addField("Laplacian smooth", 1);

	CHECK_MSTATUS_AND_RETURN_IT(addAttribute(attr_smooth_iterations));
	CHECK_MSTATUS_AND_RETURN_IT(addAttribute(attr_delta_offsets));
	CHECK_MSTATUS_AND_RETURN_IT(addAttribute(attr_deformer_mode));

	CHECK_MSTATUS_AND_RETURN_IT(attributeAffects(attr_smooth_iterations, outputGeom));
	CHECK_MSTATUS_AND_RETURN_IT(attributeAffects(attr_delta_offsets, outputGeom));
	CHECK_MSTATUS_AND_RETURN_IT(attributeAffects(attr_deformer_mode, outputGeom));

	return MStatus::kSuccess;
}

DeltaMushDeformer::DeltaMushDeformer()
{
}

MStatus DeltaMushDeformer::deform(MDataBlock& block, MItGeometry& iter, const MMatrix& mat, unsigned int multiIndex)
{
	MStatus status;

	MFnVectorArrayData fn_delta_offset(block.inputValue(attr_delta_offsets).data());

	if (!fn_delta_offset.length()) // Deformer is not bound. Ignore deformation
	{
		return MStatus::kSuccess;
	}

	MVectorArray delta_offsets = fn_delta_offset.array();

	auto input_handle = block.inputArrayValue(input, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	input_handle.jumpToElement(multiIndex);

	auto input_geo_handle = input_handle.inputValue().child(inputGeom);

	auto envelope_handle = block.inputValue(envelope, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	const float def_envelope = envelope_handle.asFloat();

	MPointArray points;
	iter.allPositions(points);
	MPointArray src_points(points);

	MFnMeshData mesh_data(input_geo_handle.data());
	MFnMesh mesh(mesh_data.object());

	// Build adjacency table. Need to fast searching of one-ring neighborhood
	std::vector<std::vector<int>> adjacency;
	adjacency.resize(mesh.numVertices());

	for (int edge_id = 0, numEdges = mesh.numEdges(); edge_id < numEdges; edge_id++)
	{
		int2 vlist;
		mesh.getEdgeVertices(edge_id, vlist);

		adjacency[vlist[0]].push_back(vlist[1]);
		adjacency[vlist[1]].push_back(vlist[0]);
	}

	std::vector<MPoint> vb1(mesh.numVertices(), MPoint()), vb2(mesh.numVertices(), MPoint());

	for (int vert_id = 0, numVerts = mesh.numVertices(); vert_id < numVerts; vert_id++)
	{
		vb1[vert_id] = points[vert_id];
	}

	auto src_vb = &vb1;
	auto dest_vb = &vb2;

	const int num_iterations = block.inputValue(attr_smooth_iterations).asInt();

	for (int i = 0; i < num_iterations; i++)
	{
		tbb::parallel_for(0, mesh.numVertices(),
			[src_vb, dest_vb, &adjacency](int vert_id)
		{
			const MPoint& base_point = (*src_vb)[vert_id];
			MVector laplacian;

			double total_weight = 0;

			for (const auto& neighbor : adjacency[vert_id])
			{
				const MPoint& vert = (*src_vb)[neighbor];
				const double dist = 1.0 / vert.distanceTo(base_point);

				laplacian = laplacian + (vert - base_point) * dist;
				total_weight += dist;
			}

			if (total_weight > 0)
			{
				(*dest_vb)[vert_id] = base_point + laplacian / total_weight;
			}
			else
			{
				(*dest_vb)[vert_id] = base_point;
			}
		});

		std::swap(src_vb, dest_vb);
	}	

	/*tbb::parallel_for(0, mesh.numVertices(),
		[=, &block, &adjacency, &mesh, &points, &delta_offsets](int vert_id)*/

	switch (block.inputValue(attr_deformer_mode).asShort())
	{
	case 0:
		{
			MVectorArray normals, tangents;
			normals.setLength(mesh.numVertices());
			tangents.setLength(mesh.numVertices());

			for (int i = 0; i < mesh.numVertices(); i++)
			{
				normals[i] = MVector();
				tangents[i] = MVector();
			}

			for (int face_id = 0, numFaces = mesh.numPolygons(); face_id < numFaces; face_id++)
			{
				MIntArray vlist;
				mesh.getPolygonVertices(face_id, vlist);

				for (int i = 0; i < vlist.length(); i++)
				{
					int id[3] =
					{
						i,
						(i + 1) % vlist.length(),
						(i - 1 >= 0) ? i - 1 : vlist.length() - 1
					};

					float central_u, central_v;
					float left_u, left_v;
					float right_u, right_v;

					mesh.getPolygonUV(face_id, id[0], central_u, central_v);
					mesh.getPolygonUV(face_id, id[1], left_u, left_v);
					mesh.getPolygonUV(face_id, id[2], right_u, right_v);

					const auto& base_point = (*src_vb)[vlist[i]];
					const auto edge1 = (*src_vb)[vlist[id[1]]] - base_point;
					const auto edge2 = (*src_vb)[vlist[id[2]]] - base_point;

					normals[vlist[i]] = normals[vlist[i]] + (edge1 ^ edge2).normal();

					float u1 = left_u - central_u;
					float u2 = right_u - central_u;
					float v1 = left_v - central_v;
					float v2 = right_v - central_v;

					float r = (u1 * v2 - u2 * v1);

					MVector tangent;

					if (fabs(r) > 10e-5)
					{
						tangent = (v2 * edge1 - v1 * edge2) / r;
					}
					else
					{
						if (fabs(u1) > 10e-5)
						{
							tangent = edge1 / u1;
						}
						else if (fabs(u2) > 10e-5)
						{
							tangent = edge2 / u2;
						}
					}

					tangents[vlist[i]] = tangents[vlist[i]] + tangent.normal();
				}
			}

			for (int vert_id = 0, numVerts = mesh.numVertices(); vert_id < numVerts; vert_id++)
			{
				const auto& base_point = (*src_vb)[vert_id];

				tangents[vert_id].normalize();

				MVector normal = normals[vert_id].normal();
				MVector tangent = (tangents[vert_id] - normal * (tangents[vert_id] * normal)).normal();

				MVector bitangent = normal ^ tangent;

				MMatrix coord_frame;

				coord_frame(0, 0) = tangent.x; coord_frame(0, 1) = tangent.y; coord_frame(0, 2) = tangent.z;
				coord_frame(1, 0) = bitangent.x; coord_frame(1, 1) = bitangent.y; coord_frame(1, 2) = bitangent.z;
				coord_frame(2, 0) = normal.x; coord_frame(2, 1) = normal.y; coord_frame(2, 2) = normal.z;
				coord_frame(3, 0) = base_point.x; coord_frame(3, 1) = base_point.y; coord_frame(3, 2) = base_point.z;

				const MPoint deformed = MPoint(delta_offsets[vert_id]) * coord_frame;
				//const MPoint deformed = base_point;

				const float weight = weightValue(block, multiIndex, vert_id) * def_envelope;
				points[vert_id] = weight * deformed + (1.0 - weight) * points[vert_id];
			}//);
		}
		break;
	case 1:
		{
			for (int vert_id = 0, numVerts = mesh.numVertices(); vert_id < numVerts; vert_id++)
			{
				const MPoint deformed = (*src_vb)[vert_id];

				const float weight = weightValue(block, multiIndex, vert_id) * def_envelope;
				points[vert_id] = weight * deformed + (1.0 - weight) * points[vert_id];
			}
		}
		break;
	}	

	iter.setAllPositions(points);

	return MStatus::kSuccess;
}
	return MStatus::kSuccess;
}
