#include "Mesh.h"

using namespace std;

void Mesh::load(const char* filename)
{
	ifstream inFile(filename); // Cleaner constructor usage
	if (!inFile) {
		cerr << "Unable to open Mesh File: " << filename << endl;
		exit(1);
	}

	string buffer;
	while (getline(inFile, buffer)) {
		stringstream ss(buffer);
		string s;
		ss >> s; // Read the first token (v, f, or nothing)

		if (s.empty() || s[0] == '#') continue;

		if (s == "v") {
			Vector3f vertex;
			ss >> vertex[0] >> vertex[1] >> vertex[2];
			bindVertices.push_back(vertex);
		}
		else if (s == "f") {
			Tuple3u face;
			ss >> face[0] >> face[1] >> face[2];

			// OBJ uses 1-based indexing, convert to 0-based
			face[0]--; face[1]--; face[2]--;
			faces.push_back(face);
		}
	}
	currentVertices = bindVertices;
}

void Mesh::draw()
{
	glBegin(GL_TRIANGLES);
	for (unsigned int i = 0; i < faces.size(); i++) {
		Vector3f v0 = currentVertices[faces[i][0]];
		Vector3f v1 = currentVertices[faces[i][1]];
		Vector3f v2 = currentVertices[faces[i][2]];

		Vector3f cross = Vector3f::cross(v1 - v0, v2 - v0);
		float len = cross.abs(); 

		
		if (len > 0.00001f) {
			Vector3f normal = cross.normalized(); 
			glNormal3d(normal.x(), normal.y(), normal.z());
			glVertex3d(v0.x(), v0.y(), v0.z());
			glVertex3d(v1.x(), v1.y(), v1.z());
			glVertex3d(v2.x(), v2.y(), v2.z());
		}
	}
	glEnd();

}

void Mesh::loadAttachments(const char* filename, int numJoints)
{
	ifstream inFile(filename);
	if (!inFile) {
		cerr << "Unable to open Attachments File: " << filename << endl;
		exit(1);
	}

	string buffer;
	while (getline(inFile, buffer)) {
		// Skip empty lines
		if (buffer.empty()) continue;

		stringstream ss(buffer);
		vector<float> weights;
		//  Pre-allocate memory
		weights.reserve(numJoints);

		float weight;
		// Some files might be malformed
		for (int i = 0; i < numJoints; i++) {
			if (ss >> weight) {
				weights.push_back(weight);
			}
			else {
				weights.push_back(0.0f);
			}
		}
		attachments.push_back(weights);
	}
}
