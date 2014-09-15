#include <maya/MArgDatabase.h>
#include <maya/MSelectionList.h>
#include <maya/MFnMesh.h>
#include <maya/MPointArray.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MPlug.h>
#include <maya/MMatrix.h>
#include <maya/MFnVectorArrayData.h>

#include <tbb/parallel_for.h>

#include <vector>
#include <tuple>

#include "deformer_node.hpp"

#include "bind_cmd.hpp"

MString BindRestCmd::cmd_name("mush_bind");

MSyntax BindRestCmd::cmd_syntax()
{
	MSyntax syntax;

	syntax.addArg(MSyntax::kString);
	syntax.addArg(MSyntax::kString);

	return syntax;
}

MStatus BindRestCmd::doIt(const MArgList& args)
{
	MStatus status;
	MArgDatabase arg_db(syntax(), args, &status);

	CHECK_MSTATUS_AND_RETURN_IT(status);

	MString target_name;
	MString target_mesh_name;
	arg_db.getCommandArgument(0, target_name);
	arg_db.getCommandArgument(1, target_mesh_name);

	MSelectionList sl;

	sl.add(target_name);
	sl.add(target_mesh_name);

	if (!sl.getDependNode(0, m_target_deformer))
	{
		displayError(MString("Could not find mush deformer with name ") + target_name);
		return MStatus::kInvalidParameter;
	}

	if (!sl.getDependNode(1, m_target_mesh))
	{
		displayError(MString("Could not find mesh with name ") + target_mesh_name);
		return MStatus::kInvalidParameter;
	}

	return redoIt();
}

MStatus BindRestCmd::redoIt()
{
	if (m_target_deformer.isNull() || m_target_mesh.isNull())
		return MStatus::kFailure;

	MFnDependencyNode def_node(m_target_deformer);

	if (def_node.typeId() != DeltaMushDeformer::type_id)
	{
		displayError("First argument is not deltaMush deformer");
		return MStatus::kInvalidParameter;
	}

	MStatus status;

	MPlug num_iterations_plug(def_node.findPlug(DeltaMushDeformer::attr_smooth_iterations, &status));
	CHECK_MSTATUS_AND_RETURN_IT(status);

	MFnMesh mesh(m_target_mesh, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	MPointArray points;
	mesh.getPoints(points); // Original mesh

	// Build adjacency table. Need to fast searching of one-ring neighborhood
	std::vector<std::vector<int>> adjacency;
	std::vector<std::vector<std::tuple<int, int>>> shared_faces;
	adjacency.resize(mesh.numVertices());
	shared_faces.resize(mesh.numVertices());

	for (int edge_id = 0, numEdges = mesh.numEdges(); edge_id < numEdges; edge_id++)
	{
		int2 vlist;
		mesh.getEdgeVertices(edge_id, vlist);

		adjacency[vlist[0]].push_back(vlist[1]);
		adjacency[vlist[1]].push_back(vlist[0]);
	}

	for (int poly_id = 0, numPolys = mesh.numPolygons(); poly_id < numPolys; poly_id++)
	{
		MIntArray vert_list;
		mesh.getPolygonVertices(poly_id, vert_list);

		for (int i = 0; i < vert_list.length(); i++)
		{
			shared_faces[vert_list[i]].push_back(std::make_tuple(poly_id, i));
		}
	}

	std::vector<MPoint> vb1(mesh.numVertices(), MPoint()), vb2(mesh.numVertices(), MPoint());

	for (int vert_id = 0, numVerts = mesh.numVertices(); vert_id < numVerts; vert_id++)
	{
		mesh.getPoint(vert_id, vb1[vert_id]);
	}

	auto src_vb = &vb1;
	auto dest_vb = &vb2;	

	for (int i = 0; i < num_iterations_plug.asInt(); i++)
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

	MVectorArray delta_offsets;
	delta_offsets.setLength(mesh.numVertices());

	/*tbb::parallel_for(0, mesh.numVertices(),
	[src_vb, dest_vb, &adjacency, &mesh, &points, &delta_offsets](int vert_id)*/	

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

		MVector offset = points[vert_id] * coord_frame.inverse();

		delta_offsets[vert_id] = offset;
	} //);

	MFnVectorArrayData fn_vec_data;

	MPlug delta_offsets_plug(def_node.findPlug(DeltaMushDeformer::attr_delta_offsets, &status));
	CHECK_MSTATUS_AND_RETURN_IT(status);

	delta_offsets_plug.setMObject(fn_vec_data.create(delta_offsets, &status));
	CHECK_MSTATUS_AND_RETURN_IT(status);

	return MStatus::kSuccess;
}

