#pragma once

#include <vector>

#include <maya/MPxDeformerNode.h>
#include <maya/MPoint.h>
#include <maya/MVector.h>

class MFnMesh;

class DeltaMushDeformer : public MPxDeformerNode
{
	enum NodeRegistrationTypeId
	{
		TypeId_Prefix = 0x7475,
		TypeId_Value = 0x7721
	};

	enum DeformerMode
	{
		Deformer_DeltaMush,
		Deformer_LaplacianSmooth,
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
	static MObject attr_rest_mesh;
	static MObject attr_weight_by_distance;

private:
	DeltaMushDeformer();

	MStatus compute(const MPlug& plug, MDataBlock& dataBlock);

	void smooth_mesh(MFnMesh& mesh, int num_iterations, bool weightByDistance, std::vector<MPoint>& points, 
		std::vector<MVector>& normals, std::vector<MVector>& tangents);
};
