//******************************************************************
// 
//  Generated by RoboCompDSL
//  
//  File name: HumanCameraBody.ice
//  Source: HumanCameraBody.idsl
//
//******************************************************************
#ifndef ROBOCOMPHUMANCAMERABODY_ICE
#define ROBOCOMPHUMANCAMERABODY_ICE
module RoboCompHumanCameraBody
{
	sequence <float> DescriptorFloat;
	sequence <byte> DescriptorByte;
	sequence <DescriptorByte> DescByteList;
	sequence <DescriptorFloat> DescFloatList;
	sequence <byte> ImgType;
	struct TImage
	{
		int width;
		int height;
		ImgType image;
	};
	struct TGroundTruth
	{
		float x;
		float y;
		float z;
		float rx;
		float ry;
		float rz;
	};
	sequence <TGroundTruth> GroundTruth;
	struct KeyPoint
	{
		float x;
		float y;
		float z;
		int i;
		int j;
		float xw;
		float yw;
		float zw;
		float score;
		DescFloatList floatdesclist;
		DescByteList bytedesclist;
	};
	dictionary <string, KeyPoint> TJoints;
	struct Person
	{
		int id;
		TJoints joints;
		TImage roi;
	};
	sequence <Person> People;
	struct PeopleData
	{
		int cameraId;
		long timestamp;
		People peoplelist;
		GroundTruth gt;
	};
	interface HumanCameraBody
	{
		PeopleData newPeopleData ();
	};
};

#endif
