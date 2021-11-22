using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.IsoXFEM.ElementStructuralStiffnessComputations
{
    public class ElementStructuralStiffnessXFEMComputation : IElementStructuralStiffnessComputation
    {
        private Matrix coordinatesOfTriangles;
        private int[,] connectionOfTriangles;
        public ElementStructuralStiffnessXFEMComputation(Matrix coordinatesOfTriangles,int[,] connectionOfTriangles)
        {
            this.coordinatesOfTriangles = coordinatesOfTriangles;
            this.connectionOfTriangles = connectionOfTriangles;
        }
        public Matrix ElementStructuralStiffnessComputation(Matrix coordinatesOfElement, Matrix elasticityMatrix, double thickness)
        {
            {               
                double x01 = coordinatesOfElement[0, 0];
                double y01 = coordinatesOfElement[0, 1];
                double x02 = coordinatesOfElement[1, 0];
                double y02 = coordinatesOfElement[1, 1];
                double x03 = coordinatesOfElement[2, 0];
                double y03 = coordinatesOfElement[2, 1];
                double x04 = coordinatesOfElement[3, 0];
                double y04 = coordinatesOfElement[3, 1];
                //calculate the transformation matrix
                Matrix A = Matrix.CreateFromArray(new double[,] { { x01 ,y01, x01 * y01, 1 } ,
                                                               {x02 ,y02 ,x02*y02, 1 },
                                                                {x03 ,y03 ,x03*y03, 1 },
                                                               {x04, y04 ,x04*y04 ,1 } });
                Matrix B = Matrix.CreateFromArray(new double[,] { { -1 ,- 1, 1, 1 },
                                                              { 1 ,-1, -1, 1} ,
                                                               { 1 ,1, 1, 1},
                                                               { -1, 1, -1, 1} });
                Matrix TM = A.Invert() * B;
                //calculate natural coordinates of triangle nodes
                Matrix tricoord = coordinatesOfTriangles;
                Matrix tricoordfirstcol = tricoord.GetSubmatrix(0, tricoord.NumRows, 0, 1);
                Matrix tricoordsecondcol = tricoord.GetSubmatrix(0, tricoord.NumRows, 1, 2);
                Matrix multElementsOfColumns = Matrix.CreateZero(tricoord.NumRows, 1);
                Matrix ones = Matrix.CreateZero(tricoord.NumRows, 1);
                for (int aa = 0; aa < tricoord.NumRows; aa++)
                {
                    multElementsOfColumns[aa, 0] = tricoordfirstcol[aa, 0] * tricoordsecondcol[aa, 0];
                    ones[aa, 0] = 1;
                }
                Matrix gcoords = tricoord.AppendRight(multElementsOfColumns).AppendRight(ones);
                Matrix TMfirstseccolumns = TM.GetSubmatrix(0, TM.NumRows, 0, 2);
                Matrix natcoords = gcoords * TMfirstseccolumns;
                //calculate gauss points of the tringles in natural coordinates
                List<Matrix> Tcoors = new List<Matrix>();
                Vector tarea = Vector.CreateZero(connectionOfTriangles.GetLength(0));
                for (int aa = 0; aa < connectionOfTriangles.GetLength(0); aa++)
                {
                    int[] triangrow = new int[connectionOfTriangles.GetLength(1)];
                    for (int bb = 0; bb < connectionOfTriangles.GetLength(1); bb++)
                    {
                        triangrow[bb] = connectionOfTriangles[aa, bb];
                    }
                    Matrix tcoor = natcoords.GetSubmatrix(triangrow, new int[] { 0, 1 });
                    int numoftricoords = tcoor.NumRows;
                    tarea[aa] = GeometryCalculations.Polygonarea(tcoor, numoftricoords);
                    Tcoors.Add(tcoor);
                }
                List<Matrix> GP = new List<Matrix>();
                Matrix gp1 = Matrix.CreateZero(2, connectionOfTriangles.GetLength(0));
                Matrix gp2 = Matrix.CreateZero(2, connectionOfTriangles.GetLength(0));
                Matrix gp3 = Matrix.CreateZero(2, connectionOfTriangles.GetLength(0));
                for (int aa = 0; aa < 2; aa++)
                {
                    for (int bb = 0; bb < connectionOfTriangles.GetLength(0); bb++)
                    {
                        Matrix tcoor = Tcoors[bb];
                        gp1[aa, bb] = (tcoor[0, aa] + tcoor[1, aa]) / 2;
                        gp2[aa, bb] = (tcoor[1, aa] + tcoor[2, aa]) / 2;
                        gp3[aa, bb] = (tcoor[2, aa] + tcoor[0, aa]) / 2;
                    }
                }
                Matrix stiffness = Matrix.CreateZero(8, 8);
                GP.Add(gp1);
                GP.Add(gp2);
                GP.Add(gp3);
                for (int aa = 0; aa < tarea.Length; aa++)
                {
                    for (int bb = 0; bb < 3; bb++)
                    {
                        Matrix gpedge = GP[bb];
                        double r = gpedge[0, aa];
                        double s = gpedge[1, aa];
                        double dN1r = -(1 - s) / 4;
                        double dN1s = -(1 - r) / 4;
                        double dN2r = (1 - s) / 4;
                        double dN2s = -(1 + r) / 4;
                        double dN3r = (1 + s) / 4;
                        double dN3s = (1 + r) / 4;
                        double dN4r = -(1 + s) / 4;
                        double dN4s = (1 - r) / 4;
                        double j11 = x01 * dN1r + x02 * dN2r + x03 * dN3r + x04 * dN4r;
                        double j12 = y01 * dN1r + y02 * dN2r + y03 * dN3r + y04 * dN4r;
                        double j21 = x01 * dN1s + x02 * dN2s + x03 * dN3s + x04 * dN4s;
                        double j22 = y01 * dN1s + y02 * dN2s + y03 * dN3s + y04 * dN4s;
                        Matrix J = Matrix.CreateFromArray(new double[,] { { j11, j12 }, { j21, j22 } });
                        double detJ = J.CalcDeterminant();
                        Matrix dNxy;
                        Matrix dNrs = Matrix.CreateFromArray(new double[,] { { dN1r, dN2r, dN3r, dN4r }, { dN1s, dN2s, dN3s, dN4s } });
                        Matrix invJ = J.Invert();
                        dNxy = invJ.MultiplyRight(dNrs);
                        double dN1x = dNxy[0, 0];
                        double dN2x = dNxy[0, 1];
                        double dN3x = dNxy[0, 2];
                        double dN4x = dNxy[0, 3];
                        double dN1y = dNxy[1, 0];
                        double dN2y = dNxy[1, 1];
                        double dN3y = dNxy[1, 2];
                        double dN4y = dNxy[1, 3];
                        Matrix BL0 = Matrix.CreateFromArray(new double[,] { { dN1x, 0, dN2x, 0, dN3x, 0, dN4x, 0 }, { 0, dN1y, 0, dN2y, 0, dN3y, 0, dN4y }, { dN1y, dN1x, dN2y, dN2x, dN3y, dN3x, dN4y, dN4x } });
                        Matrix TransposeBL0 = BL0.Transpose();
                        stiffness = stiffness + thickness * TransposeBL0 * elasticityMatrix * BL0 * (detJ / 3) * tarea[aa];
                    }
                }
                return stiffness;
            }
        }
    }
}
