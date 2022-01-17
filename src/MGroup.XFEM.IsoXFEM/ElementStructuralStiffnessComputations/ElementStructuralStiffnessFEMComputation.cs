using MGroup.LinearAlgebra.Matrices;
using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.IsoXFEM.ElementStructuralStiffnessComputations
{
   // public class ElementStructuralStiffnessFEMComputation :IElementStructuralStiffnessComputation
   // {
   //    public Matrix ElementStructuralStiffnessComputation(IIsoXfemElement element/*Matrix coordinatesOfElement, Matrix elasticityMatrix, double thickness*/)
   //     {
   //         var stiffness = Matrix.CreateZero(8, 8);
   //         //double[,] GP = new double[4, 2] { { -1 / Math.Sqrt(3), -1 / Math.Sqrt(3) }, { 1 / Math.Sqrt(3), -1 / Math.Sqrt(3) }, { 1 / Math.Sqrt(3), 1 / Math.Sqrt(3) }, { -1 / Math.Sqrt(3), 1 / Math.Sqrt(3) } };
			//Matrix coordinatesOfElement = Matrix.CreateFromArray(new double[,] {{ element.Nodes[0].X, element.Nodes[0].Y},
			//															  {element.Nodes[1].X,element.Nodes[1].Y },
			//															  {element.Nodes[2].X, element.Nodes[2].Y },
			//															  {element.Nodes[3].X, element.Nodes[3].Y }});
			//double x01 = coordinatesOfElement[0, 0];
   //         double y01 = coordinatesOfElement[0, 1];
   //         double x02 = coordinatesOfElement[1, 0];
   //         double y02 = coordinatesOfElement[1, 1];
   //         double x03 = coordinatesOfElement[2, 0];
   //         double y03 = coordinatesOfElement[2, 1];
   //         double x04 = coordinatesOfElement[3, 0];
   //         double y04 = coordinatesOfElement[3, 1];
			//var gausspoints = element.IntegrationBulk.GenerateIntegrationPoints(element);
			//for (int ii = 0; ii < 4; ii++)
   //         {
			//	//double r = GP[ii, 0];
			//	//double s = GP[ii, 1];
			//	double r = gausspoints[ii].Coordinates[0];
			//	double s = gausspoints[ii].Coordinates[1];
   //             double dN1r = -(0.25) * (1 - s);
   //             double dN1s = -0.25 * (1 - r);
   //             double dN2r = 0.25 * (1 - s);
   //             double dN2s = -0.25 * (1 + r);
   //             double dN3r = 0.25 * (1 + s);
   //             double dN3s = 0.25 * (1 + r);
   //             double dN4r = -0.25 * (1 + s);
   //             double dN4s = 0.25 * (1 - r);
   //             double j11 = x01 * dN1r + x02 * dN2r + x03 * dN3r + x04 * dN4r;
   //             double j12 = y01 * dN1r + y02 * dN2r + y03 * dN3r + y04 * dN4r;
   //             double j21 = x01 * dN1s + x02 * dN2s + x03 * dN3s + x04 * dN4s;
   //             double j22 = y01 * dN1s + y02 * dN2s + y03 * dN3s + y04 * dN4s;
   //             Matrix J = Matrix.CreateFromArray(new double[,] { { j11, j12 }, { j21, j22 } });
   //             double detJ = J.CalcDeterminant();
   //             Matrix dNxy;
   //             Matrix dNrs = Matrix.CreateFromArray(new double[,] { { dN1r, dN2r, dN3r, dN4r }, { dN1s, dN2s, dN3s, dN4s } });
   //             Matrix invJ = J.Invert();
   //             dNxy = invJ.MultiplyRight(dNrs);
   //             //dNxy= new double[2, 4] J.Invert() *{ { dN1r, dN2r, dN3r, dN4r},{ dN1s, dN2s, dN3s ,dN4s} };
   //             double dN1x = dNxy[0, 0];
   //             double dN2x = dNxy[0, 1];
   //             double dN3x = dNxy[0, 2];
   //             double dN4x = dNxy[0, 3];
   //             double dN1y = dNxy[1, 0];
   //             double dN2y = dNxy[1, 1];
   //             double dN3y = dNxy[1, 2];
   //             double dN4y = dNxy[1, 3];
   //             Matrix BL0 = Matrix.CreateFromArray(new double[,] { { dN1x, 0, dN2x, 0, dN3x, 0, dN4x, 0 }, { 0, dN1y, 0, dN2y, 0, dN3y, 0, dN4y }, { dN1y, dN1x, dN2y, dN2x, dN3y, dN3x, dN4y, dN4x } });
   //             Matrix TransposeBL0 = BL0.Transpose();
   //             Matrix firstmult = TransposeBL0.Scale(element.Thickness);
   //             Matrix secmult = firstmult.MultiplyRight(element.ElasticityMatrix);
   //             Matrix thirdmult = secmult.MultiplyRight(BL0);
   //             Matrix fourthmult = thirdmult.Scale(detJ);
   //             stiffness.AddIntoThis(fourthmult);
   //         }
   //         return stiffness;
   //     }
   // }
}
