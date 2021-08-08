using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.XFEM.Cracks;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment.Enrichers;
using MGroup.XFEM.Enrichment.SingularityResolution;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Integration;
using MGroup.XFEM.Integration.Quadratures;
using MGroup.XFEM.Materials;
using Xunit;

namespace MGroup.XFEM.Tests.Fracture.Khoei
{
	public static class Example_2_8_3
	{
		private static readonly Func<double, double> round = x => 1E6 * Math.Round(x * 1E-6, 3);

		private const double E = 2E6;
		private const double v = 0.3;
		private const double thickness = 1.0;
		private const bool planeStress = false;
		private const int subdomainID = 0;

		[Fact]
		public static void TestElementStiffnesses()
		{
			XModel<IXCrackElement> model = CreateModel();
			CreateCrack(model);
			model.Initialize();
			for (int e = 0; e < model.Elements.Count; ++e)
			{
				IMatrix computedK = model.Elements[e].StiffnessMatrix(model.Elements[e]);

				double tol = 1E-13;
				Matrix expectedK = GetExpectedStiffness(e);
				IMatrix computedRoundedK = computedK.DoToAllEntries(round);

				Assert.True(expectedK.Equals(computedRoundedK, tol));
			}
		}

		//[Fact] //TODO: Figure why this does not seem to work correctly.
		public static void TestSolution()
		{
			XModel<IXCrackElement> model = CreateModel();
			CreateCrack(model);
			model.Initialize();

			// FEM assembly
			var elementToGlobalMaps = new List<int[]>();
			elementToGlobalMaps.Add(new int[] { /*n0*/0, 1, /*n1*/2, 3, 4, 5, /*n2*/6, 7, 8, 9, /*n3*/10, 11});
			elementToGlobalMaps.Add(new int[] { /*n1*/2, 3, 4, 5, /*n4*/12, 13, 14, 15, /*n7*/20, 21, 22, 23, /*n2*/6, 7, 8, 9 });
			elementToGlobalMaps.Add(new int[] { /*n4*/12, 13, 14, 15, /*n5*/16, 17, /*n5*/18, 19, /*n7*/20, 21, 22, 23 });

			var globalK = Matrix.CreateZero(24, 24);
			for (int e = 0; e < model.Elements.Count; ++e)
			{
				IMatrix elementK = model.Elements[e].StiffnessMatrix(model.Elements[e]);
				AddSubmatrix(globalK, elementK, elementToGlobalMaps[e]);
			}

			// Solution
			int[] freeDofs =
			{ 
				/*n1*/2, 3, 4, 5, /*n2*/6, 7, 8, 9, /*n4*/12, 13, 14, 15, /*n5*/17, /*n6*/19, /*n7*/20, 21, 22, 23
			};
			int[] constrainedDofs =
			{
				/*n0*/0, 1, /*n3*/10, 11, /*n5*/16, /*n6*/18
			};
			Matrix Kff = globalK.GetSubmatrix(freeDofs, freeDofs);
			Matrix Kfc = globalK.GetSubmatrix(freeDofs, constrainedDofs);

			Vector U = GetExpectedSolution();
			Vector Uf = U.GetSubvector(freeDofs);
			Vector Uc = U.GetSubvector(constrainedDofs);

			// The system is singular or very degenerate and cannot be solved with reasonable accuracy. 
			// Instead we check that the solution satisfies it.
			Vector Ff = Kff * Uf + Kfc * Uc;
			
			// Compare
			double tol = 1E-13;
			var expectedFf = Vector.CreateFromArray(new double[freeDofs.Length]);
			Assert.True(expectedFf.Equals(Ff, tol));
		}

		private static XModel<IXCrackElement> CreateModel()
		{
			var model = new XModel<IXCrackElement>(2);
			model.Subdomains[subdomainID] = new XSubdomain(subdomainID);

			XNode[] nodes = new XNode[]
			{
				new XNode(0, 00.0, 00.0),
				new XNode(1, 20.0, 00.0),
				new XNode(2, 20.0, 20.0),
				new XNode(3, 00.0, 20.0),

				new XNode(4, 40.0, 00.0),
				new XNode(5, 60.0, 00.0),
				new XNode(6, 60.0, 20.0),
				new XNode(7, 40.0, 20.0)
			};
			foreach (XNode node in nodes)
			{
				model.Nodes[node.ID] = node;
			}

			var material = new HomogeneousFractureMaterialField2D(E, v, thickness, planeStress);
			var enrichedIntegration = new IntegrationWithNonconformingQuads2D(2, GaussLegendre2D.GetQuadratureWithOrder(2, 2));
			var bulkIntegration = new CrackElementIntegrationStrategy(
				enrichedIntegration, enrichedIntegration, enrichedIntegration);
			var factory = new XCrackElementFactory2D(material, thickness, bulkIntegration);
			var elements = new IXCrackElement[3];
			elements[0] = factory.CreateElement(0, CellType.Quad4, new XNode[] { nodes[0], nodes[1], nodes[2], nodes[3] });
			elements[1] = factory.CreateElement(1, CellType.Quad4, new XNode[] { nodes[1], nodes[4], nodes[7], nodes[2] });
			elements[2] = factory.CreateElement(2, CellType.Quad4, new XNode[] { nodes[4], nodes[5], nodes[6], nodes[7] });
			foreach (IXCrackElement element in elements)
			{
				model.Elements[element.ID] = element;
				model.Subdomains[subdomainID].Elements.Add(element);
			}

			return model;
		}

		private static void CreateCrack(XModel<IXCrackElement> model)
		{
			var geometryModel = new CrackGeometryModel(model);
			model.GeometryModel = geometryModel;
			geometryModel.Enricher = new NodeEnricherIndependentCracks(geometryModel, new NullSingularityResolver());
			var initGeometry = new PolyLine2D(new double[] { 30.0, +40.0 }, new double[] { 30.0, -40.0 });
			IPropagator propagator = null;
			var crack = new ExteriorLsmCrack(0, initGeometry, model, propagator);
			geometryModel.Cracks[crack.ID] = crack;
		}

		/// <summary>
		/// The order of dofs in node major, enrichment medium, axis minor
		/// </summary>
		/// <param name="elementID"></param>
		/// <returns></returns>
		private static Matrix GetExpectedStiffness(int elementID)
		{
			if (elementID == 0)
			{
				return 1E6 * Matrix.CreateFromArray(new double[,]
				{
					{  1.154,  0.481, -0.769,  0.096, 0.000, 0.000, -0.577, -0.481, 0.000, 0.000,  0.192, -0.096 },
					{  0.481,  1.154, -0.096,  0.192, 0.000, 0.000, -0.481, -0.577, 0.000, 0.000,  0.096, -0.769 },
					{ -0.769, -0.096,  1.154, -0.481, 0.000, 0.000,  0.192,  0.096, 0.000, 0.000, -0.577,  0.481 },
					{  0.096,  0.192, -0.481,  1.154, 0.000, 0.000, -0.096, -0.769, 0.000, 0.000,  0.481, -0.577 },
					{  0.000,  0.000,  0.000,  0.000, 0.000, 0.000,  0.000,  0.000, 0.000, 0.000,  0.000,  0.000 },
					{  0.000,  0.000,  0.000,  0.000, 0.000, 0.000,  0.000,  0.000, 0.000, 0.000,  0.000,  0.000 },
					{ -0.577, -0.481,  0.192, -0.096, 0.000, 0.000,  1.154,  0.481, 0.000, 0.000, -0.769,  0.096 },
					{ -0.481, -0.577,  0.096, -0.769, 0.000, 0.000,  0.481,  1.154, 0.000, 0.000, -0.096,  0.192 },
					{  0.000,  0.000,  0.000,  0.000, 0.000, 0.000,  0.000,  0.000, 0.000, 0.000,  0.000,  0.000 },
					{  0.000,  0.000,  0.000,  0.000, 0.000, 0.000,  0.000,  0.000, 0.000, 0.000,  0.000,  0.000 },
					{  0.192,  0.096, -0.577,  0.481, 0.000, 0.000, -0.769, -0.096, 0.000, 0.000,  1.154, -0.481 },
					{ -0.096, -0.769,  0.481, -0.577, 0.000, 0.000,  0.096,  0.192, 0.000, 0.000, -0.481,  1.154 }
				});
			}
			else if (elementID == 1)
			{
				return 1E6 * Matrix.CreateFromArray(new double[,]
				{
					{  1.154,  0.481,  0.962,  0.240, -0.769,  0.096,  0.769,  0.144, -0.577, -0.481,  0.577,  0.433,  0.192, -0.096,  0.385, -0.048 },
					{  0.481,  1.154,  0.240,  0.481, -0.096,  0.192,  0.337, -0.192, -0.481, -0.577,  0.529,  0.577,  0.096, -0.769,  0.048, -0.096 },
					{  0.962,  0.240,  1.923,  0.481, -0.769,  0.337,  0.000,  0.000, -0.577, -0.529,  0.000,  0.000,  0.385, -0.048,  0.769, -0.096 },
					{  0.240,  0.481,  0.481,  0.962,  0.144,  0.192,  0.000,  0.000, -0.433, -0.577,  0.000,  0.000,  0.048, -0.096,  0.096, -0.192 },
					{ -0.769, -0.096, -0.769,  0.144,  1.154, -0.481, -0.962,  0.240,  0.192,  0.096, -0.385, -0.048, -0.577,  0.481, -0.577,  0.433 },
					{  0.096,  0.192,  0.337,  0.192, -0.481,  1.154,  0.240, -0.481, -0.096, -0.769,  0.048,  0.096,  0.481, -0.577,  0.529, -0.577 },
					{  0.769,  0.337,  0.000,  0.000, -0.962,  0.240,  1.923, -0.481, -0.385, -0.048,  0.769,  0.096,  0.577, -0.529,  0.000,  0.000 },
					{  0.144, -0.192,  0.000,  0.000,  0.240, -0.481, -0.481,  0.962,  0.048,  0.096, -0.096, -0.192, -0.433,  0.577,  0.000,  0.000 },
					{ -0.577, -0.481, -0.577, -0.433,  0.192, -0.096, -0.385,  0.048,  1.154,  0.481, -0.962, -0.240, -0.769,  0.096, -0.769, -0.144 },
					{ -0.481, -0.577, -0.529, -0.577,  0.096, -0.769, -0.048,  0.096,  0.481,  1.154, -0.240, -0.481, -0.096,  0.192, -0.337,  0.192 },
					{  0.577,  0.529,  0.000,  0.000, -0.385,  0.048,  0.769, -0.096, -0.962, -0.240,  1.923,  0.481,  0.769, -0.337,  0.000,  0.000 },
					{  0.433,  0.577,  0.000,  0.000, -0.048,  0.096,  0.096, -0.192, -0.240, -0.481,  0.481,  0.962, -0.144, -0.192,  0.000,  0.000 },
					{  0.192,  0.096,  0.385,  0.048, -0.577,  0.481,  0.577, -0.433, -0.769, -0.096,  0.769, -0.144,  1.154, -0.481,  0.962, -0.240 },
					{ -0.096, -0.769, -0.048, -0.096,  0.481, -0.577, -0.529,  0.577,  0.096,  0.192, -0.337, -0.192, -0.481,  1.154, -0.240,  0.481 },
					{  0.385,  0.048,  0.769,  0.096, -0.577,  0.529,  0.000,  0.000, -0.769, -0.337,  0.000,  0.000,  0.962, -0.240,  1.923, -0.481 },
					{ -0.048, -0.096, -0.096, -0.192,  0.433, -0.577,  0.000,  0.000, -0.144,  0.192,  0.000,  0.000, -0.240,  0.481, -0.481,  0.962 }
				});
			}
			else if (elementID == 2)
			{
				return 1E6 * Matrix.CreateFromArray(new double[,]
				{
					{  1.154,  0.481, 0.000, 0.000, -0.769,  0.096, -0.577, -0.481,  0.192, -0.096, 0.000, 0.000 },
					{  0.481,  1.154, 0.000, 0.000, -0.096,  0.192, -0.481, -0.577,  0.096, -0.769, 0.000, 0.000 },
					{  0.000,  0.000, 0.000, 0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000, 0.000, 0.000 },
					{  0.000,  0.000, 0.000, 0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000, 0.000, 0.000 },
					{ -0.769, -0.096, 0.000, 0.000,  1.154, -0.481,  0.192,  0.096, -0.577,  0.481, 0.000, 0.000 },
					{  0.096,  0.192, 0.000, 0.000, -0.481,  1.154, -0.096, -0.769,  0.481, -0.577, 0.000, 0.000 },
					{ -0.577, -0.481, 0.000, 0.000,  0.192, -0.096,  1.154,  0.481, -0.769,  0.096, 0.000, 0.000 },
					{ -0.481, -0.577, 0.000, 0.000,  0.096, -0.769,  0.481,  1.154, -0.096,  0.192, 0.000, 0.000 },
					{  0.192,  0.096, 0.000, 0.000, -0.577,  0.481, -0.769, -0.096,  1.154, -0.481, 0.000, 0.000 },
					{ -0.096, -0.769, 0.000, 0.000,  0.481, -0.577,  0.096,  0.192, -0.481,  1.154, 0.000, 0.000 },
					{  0.000,  0.000, 0.000, 0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000, 0.000, 0.000 },
					{  0.000,  0.000, 0.000, 0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000, 0.000, 0.000 }
				});
			}
			else throw new ArgumentException();
			
		}

		private static Vector GetExpectedSolution()
		{
			return Vector.CreateFromArray(new double[]
			{
				/*n0*/0, 0, /*n1*/0, 0, 0.5, 0, /*n2*/0, 0, 0.5, 0, /*n3*/ 0, 0,
				/*n4*/1, 0, 0.5, 0, /*n5*/1, 0, /*n6*/1, 0, /*n7*/1, 0, 0.5, 0
			});
		}

		private static void AddSubmatrix(Matrix globalMatrix, IMatrix submatrix, int[] subToGlobalMatrix)
		{
			for (int j = 0; j < submatrix.NumColumns; ++j)
			{
				for (int i = 0; i < submatrix.NumRows; ++i)
				{
					globalMatrix[subToGlobalMatrix[i], subToGlobalMatrix[j]] = submatrix[i, j];
				}
			}
		}
	}
}
