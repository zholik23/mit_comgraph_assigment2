#include "SkeletalModel.h"

#include <FL/Fl.H>

using namespace std;

void SkeletalModel::load(const char *skeletonFile, const char *meshFile, const char *attachmentsFile)
{
	loadSkeleton(skeletonFile);

	m_mesh.load(meshFile);
	m_mesh.loadAttachments(attachmentsFile, m_joints.size());

	computeBindWorldToJointTransforms();
	updateCurrentJointToWorldTransforms();
}

void SkeletalModel::draw(Matrix4f cameraMatrix, bool skeletonVisible)
{
	// draw() gets called whenever a redraw is required
	// (after an update() occurs, when the camera moves, the window is resized, etc)

	m_matrixStack.clear();
	m_matrixStack.push(cameraMatrix);

	if( skeletonVisible )
	{
		drawJoints();

		drawSkeleton();
	}
	else
	{
		// Clear out any weird matrix we may have been using for drawing the bones and revert to the camera matrix.
		glLoadMatrixf(m_matrixStack.top());

		// Tell the mesh to draw itself.
		m_mesh.draw();
	}
}

void SkeletalModel::loadSkeleton(const char* filename)
{
	// Load the skeleton from file here.
	ifstream inFile;

	inFile.open(filename);
	if (!inFile) {
		cerr << "Unable to open Skeleton File";
		exit(1);   // call system to stop
	}

	string buffer;

	while (std::getline(inFile, buffer)) {
		stringstream ss(buffer);

		Joint* joint = new Joint;
		float tx, ty, tz;
		int parent;

		//get values from line
		ss >> tx;
		ss >> ty;
		ss >> tz;
		ss >> parent;


		if (parent == -1) {
			// save root joint 
			m_rootJoint = joint;
		}
		else {
			//get parent and save current joint as child
			m_joints[parent]->children.push_back(joint);
		}
		//save new joint to list of joints
		if (std::find(m_joints.begin(), m_joints.end(), joint) == m_joints.end())
			m_joints.push_back(joint);

		//save the transform relative to parent
		joint->transform = Matrix4f::translation(tx, ty, tz);
	}
}

void SkeletalModel :: drawJoints_recur(Joint *joint){

	m_matrixStack.push(joint->transform);
	for (int i = 0; i < joint->children.size(); i++) {
		drawJoints_recur(joint->children[i]);
	}
	glLoadMatrixf(m_matrixStack.top());
	glutSolidSphere(0.025f, 12, 12);
	m_matrixStack.pop();
}
void SkeletalModel::drawJoints( )
{
	// Draw a sphere at each joint. You will need to add a recursive helper function to traverse the joint hierarchy.
	//
	// We recommend using glutSolidSphere( 0.025f, 12, 12 )
	// to draw a sphere of reasonable size.
	//
	// You are *not* permitted to use the OpenGL matrix stack commands
	// (glPushMatrix, glPopMatrix, glMultMatrix).
	// You should use your MatrixStack class
	// and use glLoadMatrix() before your drawing call.

	drawJoints_recur(m_rootJoint);


}


void SkeletalModel::drawSkeleton_recur(Joint* joint) {
	m_matrixStack.push(joint->transform);
	if (joint->children.size() != 0) {
		for (int i = 0; i < joint->children.size(); i++) {
			Joint* child = joint->children[i];
			Vector3f offset = child->transform.getCol(3).xyz();
			float l = offset.abs();
			Matrix4f T = Matrix4f::translation(0, 0, 0.5);
			Matrix4f S = Matrix4f::scaling(0.025f, 0.025f, l);
			Matrix4f R;
			if (l != 0) {
				Vector3f z = offset.normalized();
				Vector3f normal = Vector3f::cross(Vector3f(0, 0, 1), z);
				R = Matrix4f::rotation(normal, acos(z.z()));
			}
			else {
				R = Matrix4f::identity();
			}
			Matrix4f boxTrans = R * S * T;
			m_matrixStack.push(boxTrans);
			glLoadMatrixf(m_matrixStack.top());
			glutSolidCube(1.0f);
			m_matrixStack.pop();
			drawSkeleton_recur(child);
		}
	}
	m_matrixStack.pop();
}

void SkeletalModel::drawSkeleton( )
{
	// Draw boxes between the joints. You will need to add a recursive helper function to traverse the joint hierarchy.
	drawSkeleton_recur(m_rootJoint);
}

void SkeletalModel::setJointTransform(int jointIndex, float rX, float rY, float rZ)
{
	// Set the rotation part of the joint's transformation matrix based on the passed in Euler angles.
}


void SkeletalModel::computeBindWorldToJointTransforms()
{
	// 2.3.1. Implement this method to compute a per-joint transform from
	// world-space to joint space in the BIND POSE.
	//
	// Note that this needs to be computed only once since there is only
	// a single bind pose.
	//
	// This method should update each joint's bindWorldToJointTransform.
	// You will need to add a recursive helper function to traverse the joint hierarchy.
}

void SkeletalModel::updateCurrentJointToWorldTransforms()
{
	// 2.3.2. Implement this method to compute a per-joint transform from
	// joint space to world space in the CURRENT POSE.
	//
	// The current pose is defined by the rotations you've applied to the
	// joints and hence needs to be *updated* every time the joint angles change.
	//
	// This method should update each joint's bindWorldToJointTransform.
	// You will need to add a recursive helper function to traverse the joint hierarchy.
}

void SkeletalModel::updateMesh()
{
	// 2.3.2. This is the core of SSD.
	// Implement this method to update the vertices of the mesh
	// given the current state of the skeleton.
	// You will need both the bind pose world --> joint transforms.
	// and the current joint --> world transforms.
}

