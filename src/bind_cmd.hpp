#pragma once

#include <maya/MPxCommand.h>
#include <maya/MSyntax.h>

class BindRestCmd : public MPxCommand
{
public:
	static MString cmd_name;
	static MSyntax cmd_syntax();

	static void* creator()
	{
		return new BindRestCmd();
	}

private:
	MObject m_target_deformer;
	MObject m_target_mesh;

	BindRestCmd()
	{

	}

	bool hasSyntax() const
	{
		return true;
	}

	bool isUndoable() const
	{
		return false;
	}

	MStatus doIt(const MArgList& args);
	MStatus redoIt();
};
