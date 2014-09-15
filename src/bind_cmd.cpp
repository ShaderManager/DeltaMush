#include <maya/MArgDatabase.h>
#include <maya/MSelectionList.h>
#include <maya/MFnMesh.h>
#include <maya/MPointArray.h>

#include <vector>

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

	MFnMesh mesh(m_target_mesh);

	MPointArray points;

	std::vector<std::vector<int>> adjacency;
	adjacency.resize(mesh.numVertices());

	for (int edge_id = 0, numEdges = mesh.numEdges(); edge_id < numEdges; edge_id++)
	{
		int2 vlist;
		mesh.getEdgeVertices(edge_id, vlist);

		adjacency[vlist[0]].push_back(vlist[1]);
		adjacency[vlist[1]].push_back(vlist[0]);
	}

	for (int i = 0; i < 2; i++)
	{
		mesh.getPoints(points);

		for (int vert_id = 0, numVerts = mesh.numVertices(); vert_id < numVerts; vert_id++)
		{
			const MPoint& base_point = points[vert_id];
			MVector laplacian;

			double total_weight = 0;

			for (const auto& neighbor : adjacency[vert_id])
			{
				const double dist = 1.0 / points[neighbor].distanceTo(points[vert_id]);

				laplacian = laplacian + (points[neighbor] - base_point) * dist;
				total_weight += dist;
			}

			mesh.setPoint(vert_id, points[vert_id] + laplacian / total_weight);
		}
	}

	mesh.updateSurface();

	return MStatus::kSuccess;
}

