#pragma once

#include <maya/MPxDeformerNode.h>

class DeltaMushDeformer : public MPxDeformerNode
{
	enum NodeRegistrationTypeId
	{
		TypeId_Prefix = 0x7475,
		TypeId_Value = 0x7721
	};

public:
	static MString type_name;
	static MTypeId type_id;

	static MStatus initializer();

	static void* creator()
	{
		return new DeltaMushDeformer();
	}

	// Attributes
	static MObject attr_smooth_iterations;
	static MObject attr_delta_offsets;
	static MObject attr_deformer_mode;

private:
	DeltaMushDeformer();

	MStatus deform(MDataBlock& block, MItGeometry& iter, const MMatrix& mat, unsigned int multiIndex);
};
