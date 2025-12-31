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

	glLoadMatrixf(m_matrixStack.top());
	glutSolidSphere(0.025f, 12, 12);

	for (int i = 0; i < joint->children.size(); i++) {
		drawJoints_recur(joint->children[i]);
	}

	m_matrixStack.pop();
}
void SkeletalModel::drawJoints( )
{

	drawJoints_recur(m_rootJoint);


}



void SkeletalModel::drawSkeleton_recur(Joint* joint) {
	m_matrixStack.push(joint->transform);

	for (int i = 0; i < joint->children.size(); i++) {
		Joint* child = joint->children[i];
		Vector3f offset = child->transform.getCol(3).xyz();
		float l = offset.abs();

		// Calculate transformations for the "Bone" (Cube)
		Matrix4f T = Matrix4f::translation(0, 0, 0.5);
		Matrix4f S = Matrix4f::scaling(0.025f, 0.025f, l);
		Matrix4f R;

		if (l > 0.0001f) {
			Vector3f z = offset.normalized();
			Vector3f up(0, 0, 1);
			Vector3f normal = Vector3f::cross(up, z);

			if (normal.abs() < 0.0001f) {

				if (z.z() < 0) {
					R = Matrix4f::rotateX(3.14159f); // Flip 180 degrees
				}
				else {
					R = Matrix4f::identity(); // Already aligned
				}
			}
			else {
				R = Matrix4f::rotation(normal, acos(z.z()));
			}
		}
		else {
			R = Matrix4f::identity();
		}

		// Draw the bone connecting current joint to child
		Matrix4f boxTrans = R * S * T;
		m_matrixStack.push(boxTrans);
		glLoadMatrixf(m_matrixStack.top());
		glutSolidCube(1.0f);
		m_matrixStack.pop();

		// Continue recursion
		drawSkeleton_recur(child);
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
	Matrix4f b = m_joints[jointIndex]->transform;
	b.setSubmatrix3x3(0, 0, (Matrix4f::rotateZ(rZ) * Matrix4f::rotateY(rY) * Matrix4f::rotateX(rX)).getSubmatrix3x3(0,0));
	m_joints[jointIndex]->transform = b;
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
	m_matrixStack.clear();
	
	computeBindWorldToJointTransforms_recur(m_rootJoint);

}
void SkeletalModel::computeBindWorldToJointTransforms_recur(Joint* joint) {
	m_matrixStack.push(joint->transform);


	joint->bindWorldToJointTransform = m_matrixStack.top().inverse();

	for (int i = 0; i < joint->children.size(); i++) {
		computeBindWorldToJointTransforms_recur(joint->children[i]);
	}
	m_matrixStack.pop();
}

void SkeletalModel::updateCurrentJointToWorldTransforms()
{
	m_matrixStack.clear();
	// Start recursion directly on root
	updateCurrentJointToWorldTransforms_recur(m_rootJoint);
}
void SkeletalModel::updateCurrentJointToWorldTransforms_recur(Joint* joint) {
	m_matrixStack.push(joint->transform);
	joint->currentJointToWorldTransform = m_matrixStack.top();
	for (int i = 0; i < joint->children.size(); i++) {
		Joint* child = joint->children[i];
		updateCurrentJointToWorldTransforms_recur(child);
	}
	m_matrixStack.pop();

}
void SkeletalModel::updateMesh()
{
	vector<Vector3f> newVertices;
	newVertices.reserve(m_mesh.bindVertices.size()); // reserve memory

	for (int i = 0; i < m_mesh.bindVertices.size(); i++) {
		Vector4f sumpoint(0, 0, 0, 0);
		Vector4f p(m_mesh.bindVertices[i], 1); // Homogeneous coordinate

		for (int j = 0; j < m_joints.size(); j++) {
			float w = m_mesh.attachments[i][j];
				if (w > 0.00001f) {
				
				Vector4f transformed = m_joints[j]->currentJointToWorldTransform * m_joints[j]->bindWorldToJointTransform * p;
				sumpoint = sumpoint + transformed * w;
			}
		}
		newVertices.push_back(sumpoint.xyz());
	}
	m_mesh.currentVertices = newVertices;
}

