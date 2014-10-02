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

#include <tbb/parallel_for.h>

#include "deformer_node.hpp"

MString DeltaMushDeformer::type_name("deltaMush");
MTypeId DeltaMushDeformer::type_id(DeltaMushDeformer::TypeId_Prefix, DeltaMushDeformer::TypeId_Value);

MObject DeltaMushDeformer::attr_smooth_iterations;
MObject DeltaMushDeformer::attr_delta_offsets;
MObject DeltaMushDeformer::attr_deformer_mode;
MObject DeltaMushDeformer::attr_rest_mesh;
MObject DeltaMushDeformer::attr_weight_by_distance;

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

	attr_deformer_mode = enum_attr.create("mode", "mode", Deformer_DeltaMush, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	enum_attr.addField("Delta Mush", Deformer_DeltaMush);
	enum_attr.addField("Laplacian smooth", Deformer_LaplacianSmooth);

	attr_weight_by_distance = num_attr.create("weightByDistance", "diwt", MFnNumericData::kBoolean, 1, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	attr_rest_mesh = typed_attr.create("restMesh", "rest", MFnData::kMesh, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	typed_attr.setHidden(true);

	CHECK_MSTATUS_AND_RETURN_IT(addAttribute(attr_smooth_iterations));
	CHECK_MSTATUS_AND_RETURN_IT(addAttribute(attr_delta_offsets));
	CHECK_MSTATUS_AND_RETURN_IT(addAttribute(attr_deformer_mode));
	CHECK_MSTATUS_AND_RETURN_IT(addAttribute(attr_rest_mesh));
	CHECK_MSTATUS_AND_RETURN_IT(addAttribute(attr_weight_by_distance));

	CHECK_MSTATUS_AND_RETURN_IT(attributeAffects(attr_smooth_iterations, outputGeom));
	CHECK_MSTATUS_AND_RETURN_IT(attributeAffects(attr_delta_offsets, outputGeom));
	CHECK_MSTATUS_AND_RETURN_IT(attributeAffects(attr_deformer_mode, outputGeom));
	CHECK_MSTATUS_AND_RETURN_IT(attributeAffects(attr_rest_mesh, outputGeom));
	CHECK_MSTATUS_AND_RETURN_IT(attributeAffects(attr_rest_mesh, attr_delta_offsets));
	CHECK_MSTATUS_AND_RETURN_IT(attributeAffects(attr_smooth_iterations, attr_delta_offsets));
	CHECK_MSTATUS_AND_RETURN_IT(attributeAffects(attr_weight_by_distance, attr_delta_offsets));
	CHECK_MSTATUS_AND_RETURN_IT(attributeAffects(attr_weight_by_distance, outputGeom));

	return MStatus::kSuccess;
}

DeltaMushDeformer::DeltaMushDeformer()
{
}

void DeltaMushDeformer::smooth_mesh(MFnMesh& mesh, int num_iterations, bool weightByDistance, std::vector<MPoint>& points, 
									std::vector<MVector>& normals, std::vector<MVector>& tangents)
{
	// Build adjacency table. Used for fast searching of one-ring neighborhood
	std::vector<std::vector<int>> adjacency;
	adjacency.resize(mesh.numVertices());

	for (int edge_id = 0, numEdges = mesh.numEdges(); edge_id < numEdges; edge_id++)
	{
		int2 vlist;
		mesh.getEdgeVertices(edge_id, vlist);

		adjacency[vlist[0]].push_back(vlist[1]);
		adjacency[vlist[1]].push_back(vlist[0]);
	}

	std::vector<MPoint> temp_vb(mesh.numVertices(), MPoint());

	auto src_vb = &points;
	auto dest_vb = &temp_vb;

	for (int i = 0; i < num_iterations; i++)
	{
		tbb::parallel_for(0, mesh.numVertices(),
			[src_vb, dest_vb, &adjacency, weightByDistance](int vert_id)
		{
			const MPoint& base_point = (*src_vb)[vert_id];
			MVector laplacian;

			double total_weight = 0;

			for (const auto& neighbor : adjacency[vert_id])
			{
				const MPoint& vert = (*src_vb)[neighbor];				

				if (weightByDistance)
				{
					double edge_cost = vert.distanceTo(base_point); // Weight by inverse distance. It is called Modified Laplacian Smoothing

					if (edge_cost < 10e-5)
						continue;

					edge_cost = 1.0 / edge_cost;

					//laplacian = laplacian + (vert - base_point) * edge_cost;
					laplacian = laplacian + vert * edge_cost;
					total_weight += edge_cost;
				}
				else
				{
					laplacian = laplacian + vert;
					total_weight += 1.0;
				}		
			}

			if (total_weight > 0)
			{
				(*dest_vb)[vert_id] = laplacian / total_weight;
			}
			else
			{
				(*dest_vb)[vert_id] = base_point;
			}
		});

		std::swap(src_vb, dest_vb);
	}

	points = std::move(*src_vb);

	normals.resize(mesh.numVertices());
	tangents.resize(mesh.numVertices());

	MIntArray vlist;

	for (int face_id = 0, numFaces = mesh.numPolygons(); face_id < numFaces; face_id++)
	{		
		mesh.getPolygonVertices(face_id, vlist);

		for (int i = 0; i < vlist.length(); i++)
		{
			const int id[3] =
			{
				i,
				(i + 1) % vlist.length(),
				(i - 1 >= 0) ? i - 1 : vlist.length() - 1
			};			

			const auto& base_point = points[vlist[id[0]]];
			const auto edge1 = points[vlist[id[1]]] - base_point;
			const auto edge2 = points[vlist[id[2]]] - base_point;

			MVector tangent;
			//MVector normal = ((edge1 * edge2) * (edge1 ^ edge2)).normal();
			MVector normal = ((edge1 ^ edge2)).normal();

			normals[vlist[i]] = normals[vlist[i]] + normal;

			//tangent = (edge1 - normal * (edge1 * normal)).normal();

			float central_u, central_v;
			float left_u, left_v;
			float right_u, right_v;

			mesh.getPolygonUV(face_id, id[0], central_u, central_v);
			mesh.getPolygonUV(face_id, id[1], left_u, left_v);
			mesh.getPolygonUV(face_id, id[2], right_u, right_v);

			float u1 = left_u - central_u;
			float u2 = right_u - central_u;
			float v1 = left_v - central_v;
			float v2 = right_v - central_v;

			float r = (u1 * v2 - u2 * v1);			

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
}

MStatus DeltaMushDeformer::compute(const MPlug& plug, MDataBlock& dataBlock)
{
	MStatus status;

	if (plug == attr_delta_offsets)
	{
		MDataHandle rest_mesh_handle = dataBlock.inputValue(attr_rest_mesh);

		if (rest_mesh_handle.data().isNull())
		{
			return MStatus::kFailure;
		}

		const bool weight_by_dist = dataBlock.inputValue(attr_weight_by_distance).asBool();

		MDataHandle delta_offsets_handle = dataBlock.outputValue(attr_delta_offsets);

		MFnMeshData mesh_data(rest_mesh_handle.data());
		MFnMesh mesh(mesh_data.object());

		std::vector<MPoint> points;
		std::vector<MVector> normals, tangents;

		points.resize(mesh.numVertices());

		for(int vert_id = 0, numVerts = mesh.numVertices(); vert_id < numVerts; vert_id++)
		{
			mesh.getPoint(vert_id, points[vert_id]);
		}

		const int num_iterations = dataBlock.inputValue(attr_smooth_iterations).asInt();

		smooth_mesh(mesh, num_iterations, weight_by_dist, points, normals, tangents);
		
		MVectorArray delta_offsets;		
		delta_offsets.setLength(mesh.numVertices());

		tbb::parallel_for(0, mesh.numVertices(), 
		[&points, &tangents, &normals, &delta_offsets, &mesh](int vert_id)
		{
			const auto& base_point = points[vert_id];

			tangents[vert_id].normalize();

			MVector normal = normals[vert_id].normal();

			// Orthonormalize through Gram–Schmidt process
			MVector tangent = (tangents[vert_id] - normal * (tangents[vert_id] * normal)).normal();
			MVector bitangent = tangent ^ normal;

			MMatrix coord_frame;

			coord_frame(0, 0) = tangent.x; coord_frame(0, 1) = tangent.y; coord_frame(0, 2) = tangent.z;
			coord_frame(1, 0) = bitangent.x; coord_frame(1, 1) = bitangent.y; coord_frame(1, 2) = bitangent.z;
			coord_frame(2, 0) = normal.x; coord_frame(2, 1) = normal.y; coord_frame(2, 2) = normal.z;
			coord_frame(3, 0) = base_point.x; coord_frame(3, 1) = base_point.y; coord_frame(3, 2) = base_point.z;

			MPoint origin;
			mesh.getPoint(vert_id, origin);

			MVector offset = origin * coord_frame.inverse();

			delta_offsets[vert_id] = offset;
		});		

		MFnVectorArrayData fn_vec_data;
		delta_offsets_handle.setMObject(fn_vec_data.create(delta_offsets, &status));
		CHECK_MSTATUS_AND_RETURN_IT(status);		

		dataBlock.setClean(plug);

		return MStatus::kSuccess;
	}
	else if (plug == outputGeom)
	{
		unsigned int index = plug.logicalIndex();
		MPlug input_plug(thisMObject(), input);
		status = input_plug.selectAncestorLogicalIndex(index, input);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle input_handle = dataBlock.inputValue(input_plug, &status);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MDataHandle inputGeom_handle = input_handle.child(inputGeom);
		MDataHandle group_handle = input_handle.child(groupId);
		unsigned int groupId = group_handle.asLong();
		MDataHandle outputGeom_handle = dataBlock.outputValue(plug);
		status = outputGeom_handle.copy(inputGeom_handle);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MItGeometry iter(outputGeom_handle, groupId, false);

		int t1 = iter.count();		

		MPointArray src_points;
		status = iter.allPositions(src_points);
		CHECK_MSTATUS_AND_RETURN_IT(status);

		MFnVectorArrayData fn_delta_offset(dataBlock.inputValue(attr_delta_offsets).data());
		
		if (fn_delta_offset.length()) // Deformer is not bound. Ignore deformation
		{			
			MVectorArray delta_offsets = fn_delta_offset.array();

			auto envelope_handle = dataBlock.inputValue(envelope, &status);
			CHECK_MSTATUS_AND_RETURN_IT(status);

			const float def_envelope = envelope_handle.asFloat();						

			const int num_iterations = dataBlock.inputValue(attr_smooth_iterations).asInt();

			std::vector<MPoint> points;
			std::vector<MVector> normals, tangents;

			MFnMeshData mesh_data(inputGeom_handle.data());
			MFnMesh mesh(mesh_data.object());

			points.resize(mesh.numVertices());

			int t1 = src_points.length();
			for(int vert_id = 0, numVerts = mesh.numVertices(); vert_id < numVerts; vert_id++)
			{
				points[vert_id] = src_points[vert_id];
			}

			const bool weight_by_dist = dataBlock.inputValue(attr_weight_by_distance).asBool();

			smooth_mesh(mesh, num_iterations, weight_by_dist, points, normals, tangents);			

			const short deformer_mode = dataBlock.inputValue(attr_deformer_mode).asShort();

			switch (deformer_mode)
			{
			case Deformer_DeltaMush:
				{	
					tbb::parallel_for(0, mesh.numVertices(),
					[=, &points, &normals, &tangents, &dataBlock](int vert_id)
					{
						const auto& base_point = points[vert_id];

						tangents[vert_id].normalize();

						MVector normal = normals[vert_id].normal();
						MVector tangent = (tangents[vert_id] - normal * (tangents[vert_id] * normal)).normal();

						MVector bitangent = tangent ^ normal;

						MMatrix coord_frame;

						coord_frame(0, 0) = tangent.x;	  coord_frame(0, 1) = tangent.y;	coord_frame(0, 2) = tangent.z;
						coord_frame(1, 0) = bitangent.x;  coord_frame(1, 1) = bitangent.y;  coord_frame(1, 2) = bitangent.z;
						coord_frame(2, 0) = normal.x;	  coord_frame(2, 1) = normal.y;		coord_frame(2, 2) = normal.z;
						coord_frame(3, 0) = base_point.x; coord_frame(3, 1) = base_point.y; coord_frame(3, 2) = base_point.z;

						const MPoint deformed = MPoint(delta_offsets[vert_id]) * coord_frame;

						const float weight = weightValue(dataBlock, groupId, vert_id) * def_envelope;
						points[vert_id] = weight * deformed + (1.0 - weight) * src_points[vert_id];
					});		
				}
				break;
			case Deformer_LaplacianSmooth:
				{
					tbb::parallel_for(0, mesh.numVertices(),
						[=, &points, &src_points, &normals, &tangents, &dataBlock](int vert_id)
					{
						const float weight = weightValue(dataBlock, groupId, vert_id) * def_envelope;
						points[vert_id] = weight * points[vert_id] + (1.0 - weight) * src_points[vert_id];
					});					
				}
				break;
			}	

			for(int vert_id = 0, numVerts = mesh.numVertices(); vert_id < numVerts; vert_id++)
			{
				src_points[vert_id] = points[vert_id];
			}			
		}

		iter.setAllPositions(src_points);

		dataBlock.setClean(plug);

		return MStatus::kSuccess;
	}

	return MStatus::kUnknownParameter;
}
