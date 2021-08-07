SetFactory("OpenCASCADE");
Mesh.MshFileVersion = 2.2;

// *******************************************************************************************
// PARAMETERS
// *******************************************************************************************
// Domain Parameters
xmin = { -1, -1, -1 };
xmax = { +1, +1, +1 };

// Inclusion Parameters
numInclusions = 8;
xIncl = { -0.8, +0.5, +0.8, -0.5, -0.5, +0.5, +0.5, -0.5 };
yIncl = { -0.8, -0.9, +0.5, +0.5, -0.5, -0.5, +0.5, +0.5 };
zIncl = { -0.8, -0.8, -0.5, -0.5, +0.5, +0.5, +0.5, +0.5 };
rIncl = { 0.35, 0.30, 0.45, 0.40, 0.25, 0.20, 0.35, 0.30 };

// Physical Groups
matrix = 1;
inclusions = 2;

// *******************************************************************************************
// GEOMETRIC ENTITIES
// *******************************************************************************************
// This will preserve the physical group tags of the spherical inclusions after Constructive Solid Geometry operations
Geometry.OCCBooleanPreserveNumbering = 1;

// Domain
Box(1) = {xmin[0], xmin[1], xmin[2], xmax[0]-xmin[0], xmax[1]-xmin[1], xmax[2]-xmin[2]};

//Inclusions
For i In { 0 : numInclusions-1 }
	id = 2 + i;
	tempID = 10 * numInclusions + i;
	Sphere(tempID) = {xIncl[i], yIncl[i], zIncl[i], rIncl[i]};
	BooleanIntersection(id) = { Volume{tempID}; Delete; }{ Volume{1}; };
EndFor
Physical Volume(inclusions) = {2:numInclusions+1};

// BooleanFragments will intersect the box with the spherical inclusions, in a conformal manner (without creating duplicate interfaces). The original volumes must be deleted.
volumes() = BooleanFragments{ Volume{1}; Delete; }{ Volume{2:numInclusions+1}; Delete; };

// The physical volume tags of the original spheres are retained during BooleanFragments. However, the physical volume tag of the outside box must be set (actually I am not sure that it would not be retained as well, but tutorial16.geo did it this way). This is the last entry in the volumes that BooleanFragments outputs.
Physical Volume(matrix) = volumes(#volumes()-1); // The symbol # returns the length of a list
 
// *******************************************************************************************
// MESH SIZES
// *******************************************************************************************

// Adapt mesh sizes with respect to curvature of geometric entities.
Mesh.MeshSizeFromCurvature = 12; // This is the number of line segments used to approximate 2*pi radians

// Use elements with quadratic shape functions
//Mesh.ElementOrder = 2;
//Mesh.HighOrderOptimize = 2;


// Domain corners mesh size
//h_matr = 0.5;
//Point(newp) = { xmin[0], xmin[1], xmin[2], h_matr };
//Point(newp) = { xmax[0], xmin[1], xmin[2], h_matr };
//Point(newp) = { xmax[0], xmax[1], xmin[2], h_matr };
//Point(newp) = { xmin[0], xmax[1], xmin[2], h_matr };
//Point(newp) = { xmin[0], xmin[1], xmax[2], h_matr };
//Point(newp) = { xmax[0], xmin[1], xmax[2], h_matr };
//Point(newp) = { xmax[0], xmax[1], xmax[2], h_matr };
//Point(newp) = { xmin[0], xmax[1], xmax[2], h_matr };

// Inclusion centers mesh size
//h_incl = 0.01;
//Point(newp) = {+0.5, +0.5, +0.5, h_incl };
//Point(newp) = {-0.5, +0.5, +0.5, h_incl };


// These next lines are used to eliminate duplicate nodes. They are not necessary for this model.
//Mesh 3;
//Coherence Mesh;