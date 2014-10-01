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

	MSelectionList sl;

	sl.add(target_name);

	if (!sl.getDependNode(0, m_target_deformer))
	{
		displayError(MString("Could not find mush deformer with name ") + target_name);
		return MStatus::kInvalidParameter;
	}

	return redoIt();
}

MStatus BindRestCmd::redoIt()
{
	if (m_target_deformer.isNull())
		return MStatus::kFailure;

	MFnDependencyNode def_node(m_target_deformer);

	if (def_node.typeId() != DeltaMushDeformer::type_id)
	{
		displayError("First argument is not deltaMush deformer");
		return MStatus::kInvalidParameter;
	}

	MStatus status;

	MPlug input_plug = def_node.findPlug(DeltaMushDeformer::input, &status);	

	CHECK_MSTATUS_AND_RETURN_IT(status);

	input_plug = input_plug.elementByPhysicalIndex(0, &status);
	// status = input_plug.selectAncestorLogicalIndex(0, DeltaMushDeformer::input);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	MPlug input_geo_plug = input_plug.child(DeltaMushDeformer::inputGeom, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	MPlug rest_mesh_plug = def_node.findPlug(DeltaMushDeformer::attr_rest_mesh, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);

	rest_mesh_plug.setMObject(input_geo_plug.asMObject());

	return MStatus::kSuccess;
}

