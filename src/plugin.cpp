#include <maya/MFnPlugin.h>

#include "bind_cmd.hpp"
#include "deformer_node.hpp"

template<typename CommandClass> MStatus register_cmd(MFnPlugin& plugin)
{
	return plugin.registerCommand(CommandClass::cmd_name, CommandClass::creator, CommandClass::cmd_syntax);
}

MStatus initializePlugin(MObject plugin_obj)
{
	MStatus status;
	MFnPlugin plugin(plugin_obj, "_ShaMan_", "0.1", "Any", &status);

	CHECK_MSTATUS_AND_RETURN_IT(status);

	status = register_cmd<BindRestCmd>(plugin);

	CHECK_MSTATUS_AND_RETURN_IT(status);

	status = plugin.registerNode(DeltaMushDeformer::type_name, DeltaMushDeformer::type_id, DeltaMushDeformer::creator, DeltaMushDeformer::initializer, MPxNode::kDeformerNode);

	return MStatus::kSuccess;
}

MStatus uninitializePlugin(MObject plugin_obj)
{
	MFnPlugin plugin(plugin_obj);

	CHECK_MSTATUS_AND_RETURN_IT(plugin.deregisterCommand(BindRestCmd::cmd_name));

	return MStatus::kSuccess;
}
