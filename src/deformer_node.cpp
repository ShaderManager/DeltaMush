#include "deformer_node.hpp"

MString DeltaMushDeformer::type_name("deltaMush");
MTypeId DeltaMushDeformer::type_id(DeltaMushDeformer::TypeId_Prefix, DeltaMushDeformer::TypeId_Value);

MStatus DeltaMushDeformer::initializer()
{
	return MStatus::kSuccess;
}

DeltaMushDeformer::DeltaMushDeformer()
{}

MStatus DeltaMushDeformer::deform(MDataBlock& block, MItGeometry& iter, const MMatrix& mat, unsigned int multiIndex)
{
	return MStatus::kSuccess;
}
