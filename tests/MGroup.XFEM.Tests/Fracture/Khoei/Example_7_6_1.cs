using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using MGroup.Constitutive.Structural;
using MGroup.LinearAlgebra.Distributed;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.MSolve.Meshes.Structured;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.NumericalAnalyzers;
using MGroup.Solvers.AlgebraicModel;
using MGroup.Solvers.Direct;
using MGroup.XFEM.Cracks;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Cracks.Jintegral;
using MGroup.XFEM.Cracks.PropagationCriteria;
using MGroup.XFEM.Cracks.PropagationTermination;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment;
using MGroup.XFEM.Enrichment.Enrichers;
using MGroup.XFEM.Enrichment.Functions;
using MGroup.XFEM.Enrichment.SingularityResolution;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry;
using MGroup.XFEM.Geometry.Mesh;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Integration;
using MGroup.XFEM.Integration.Quadratures;
using MGroup.XFEM.Materials;
using MGroup.XFEM.Tests.Utilities;
using Xunit;

namespace MGroup.XFEM.Tests.Fracture.Khoei
{
	public static class Example_7_6_1
	{
		private static readonly double[] minCoords = new double[] { 0, 0 };
		private static readonly double[] maxCoords = new double[] { 60, 20 };
		private const double thickness = 1.0;
		private const double E = 2E6, v = 0.3;
		private const int subdomainID = 0;

		[Fact]
		public static void TestSolution3x1()
		{
			// Dof numbering
			// node 0: ux_0=con uy_0=con
			// node 1: ux_1=0  uy_1=1  t0x_1=2  t0y_1=3  t1x_1=4  t1y_1=5  t2x_1=6  t2y_1=7  t3x_1=8  t3y_1=9
			// node 2: ux_2=10 uy_2=11 t0x_2=12 t0y_2=13 t1x_2=14 t1y_2=15 t2x_2=16 t2y_2=17 t3x_1=18 t3y_2=19
			// node 3: ux_3=con uy_3=con
			// node 4: ux_4=20 uy_1=21 t0x_4=22 t0y_4=23 t1x_4=24 t1y_4=25 t2x_4=26 t2y_4=27 t3x_1=28 t3y_4=29 
			// node 5: ux_5=30 uy_5=con hx_5=31  hy_5=32
			// node 6: ux_6=33 uy_6=con hx_6=34  hy_6=35
			// node 7: ux_7=36 uy_1=37 t0x_7=38 t0y_7=39 t1x_7=40 t1y_7=41 t2x_7=42 t2y_7=43 t3x_1=44 t3y_7=45 

			var expectedStdDisplacements = 1E-3 * Vector.CreateFromArray(new double[] 
			{ 
				-8.17, -50, -8.17, 50 // ux_5 uy_5 ux_6 uy_6
			}); 
			var expectedEnrDisplacements = 1E-3 * Vector.CreateFromArray(new double[]
			{
				15.69, 49.88, -15.69, 49.88 // hx_5 hy_5 hx_6 hy_6
			}); 

			// Create and analyze model, in order to get the solution vector
			XModel<IXCrackElement> model = CreateModel3x1();
			model.Initialize();
			IDofType dofStdX = StructuralDof.TranslationX;
			IDofType dofStdY = StructuralDof.TranslationY;
			IDofType dofStepX = model.Enrichments[0].EnrichedDofs[0];
			IDofType dofStepY = model.Enrichments[0].EnrichedDofs[1];
			(IAlgebraicModel algebraicModel, IGlobalVector globalU, IMatrixView globalK) = RunAnalysis(model);

			var computedStdDisplacements = Vector.CreateFromArray(new double[]
			{
				algebraicModel.ExtractSingleValue(globalU, model.Nodes[5], dofStdX), // ux_5 
				-50E-3, // uy_5 
				algebraicModel.ExtractSingleValue(globalU, model.Nodes[6], dofStdX), // ux_6 
				+50E-3 // uy_6 
			});
			var computedEnrDisplacements = Vector.CreateFromArray(new double[]
			{
				algebraicModel.ExtractSingleValue(globalU, model.Nodes[5], dofStepX), // hx_5 
				algebraicModel.ExtractSingleValue(globalU, model.Nodes[5], dofStepY), // hy_5 
				algebraicModel.ExtractSingleValue(globalU, model.Nodes[6], dofStepX), // hx_6 
				algebraicModel.ExtractSingleValue(globalU, model.Nodes[6], dofStepY)  // hy_6
			});

			// Check
			double tol = 1E-13;
			Func<double, double> round = x => 1E-3 * Math.Round(x * 1E3, 2);
			Assert.True(expectedStdDisplacements.Equals(computedStdDisplacements.DoToAllEntries(round), tol));
			Assert.True(expectedEnrDisplacements.Equals(computedEnrDisplacements.DoToAllEntries(round), tol));
		}

		[Fact]
		public static void TestSolution135x45()
		{
			// Expected displacements of the element containing the crack mouth
			// Dofs: ux0 uy0 hx0 hy0 ux1 uy1 hx1 hy1 ux2 uy2 hx2 hy2 ux3 uy3 hx3 hy3
			var expectedDisplacements = 1E-3 * Vector.CreateFromArray(new double[]
			{
				//ux1 uy1     ux2   uy2    hx1   hy1    hx2    hy2 
				9.12, -48.17, 9.12, 48.17, 0.43, 48.17, -0.43, 48.17
			});
			int[] expectedDisplacementDofs = { 4, 5, 8, 9, 6, 7, 10, 11 };

			// Create and analyze model, in order to get the solution vector
			int[] numElements = { 135, 45 };
			XModel<IXCrackElement> model = CreateModel(numElements);
			model.Initialize();
			var crack = (Crack)model.GeometryModel.GetDiscontinuity(0);
			(IAlgebraicModel algebraicModel, IGlobalVector globalU, IMatrixView globalK) = RunAnalysis(model);

			// Find displacements at specified dofs
			IXFiniteElement mouthElement = model.Elements[crack.MouthElementID];
			double[] elementDisplacements = algebraicModel.ExtractElementVector(globalU, mouthElement);
			Vector computedDisplacements = Vector.CreateFromArray(elementDisplacements).GetSubvector(expectedDisplacementDofs);

			// Check
			double tol = 1E-13;
			Func<double, double> round = x => 1E-3 * Math.Round(x * 1E3, 2);
			Assert.True(expectedDisplacements.Equals(computedDisplacements.DoToAllEntries(round), tol));
		}

		[Fact]
		public static void TestStiffnesses3x1()
		{
			Matrix node6StiffnessExpected = 1E6 * Matrix.CreateFromArray(new double[,]
			{
				{ 1.154, 0.481, -0.481, -0.240 },
				{ 0.481, 1.154, -0.240, -0.962 },
				{ -0.481, -0.240, 0.962, 0.481 },
				{ -0.240, -0.962, 0.481, 1.923 }
			});

			Matrix node7Elem1StiffnessExpected = 1E6 * Matrix.CreateFromArray(new double[,]
			{
				{ 1.154, 0.481, 1.568, 0.544, -0.444, 0.12, -0.847, 0.016, 0.378, -0.337 },
				{ 0.481, 1.154, 0.575, 2.668, -0.114, 0.175, -0.165, -0.271, -0.055, -0.358 },
				{ 1.568, 0.575, 12.432, 4.896, -0.824, -1.69, -3.114, -3.125, 0.134, 0.537 },
				{ 0.544, 2.668, 4.896, 17.018, -1.359, -3.322, -2.444, -6.366, 0.465, 3.07 },
				{ -0.444, -0.114, -0.824, -1.359, 2.639, -0.459, 1.648, -0.253, -1.869, 0.939 },
				{ 0.12, 0.175, -1.69, -3.322, -0.459, 3.921, -0.214, 4.699, 0.909, -2.729 },
				{ -0.847, -0.165, -3.114, -2.444, 1.648, -0.214, 4.063, -0.01, -1.386, 0.645 },
				{ 0.016, -0.271, -3.125, -6.366, -0.253, 4.699, -0.01, 7.896, 0.685, -2.804 },
				{ 0.378, -0.055, 0.134, 0.465, -1.869, 0.909, -1.386, 0.685, 3.081, -0.859 },
				{ -0.337, -0.358, 0.537, 3.07, 0.939, -2.729, 0.645, -2.804, -0.859, 4.694 }
			});

			Matrix node7Elem2StiffnessExpected = 1E6 * Matrix.CreateFromArray(new double[,]
			{
				{ 1.154, -0.481, 1.715, -0.752, -0.586, 0.006, -0.886, -0.052, 0.600, -0.087 },
				{ -0.481, 1.154, -0.856, 3.184, 0.092, -0.325, 0.106, -0.417, -0.156, 0.417 },
				{ 1.715, -0.856, 13.871, -5.198, -2.050, 0.718, -3.537, 1.216, 1.561, -0.610 },
				{ -0.752, 3.184, -5.198, 26.465, 0.679, -5.280, 1.135, -9.091, -0.753, 3.713 },
				{ -0.586, 0.092, -2.050, 0.679, 1.098, -0.169, 1.822, -0.235, -0.843, 0.223 },
				{ 0.006, -0.325, 0.718, -5.280, -0.169, 2.052, -0.246, 3.583, 0.234, -1.222 },
				{ -0.886, 0.106, -3.537, 1.135, 1.822, -0.246, 3.075, -0.350, -1.317, 0.322 },
				{ -0.052, -0.417, 1.216, -9.091, -0.235, 3.583, -0.350, 6.348, 0.330, -1.973 },
				{ 0.600, -0.156, 1.561, -0.753, -0.843, 0.234, -1.317, 0.330, 0.869, -0.282 },
				{ -0.087, 0.417, -0.610, 3.713, 0.223, -1.222, 0.322, -1.973, -0.282, 1.180 }
			});

			Matrix node7GlobalStiffnessExpected = 1E6 * Matrix.CreateFromArray(new double[,]
			{
				{ 2.308, 0.000, 3.283, -0.208, -1.030, 0.126, -1.733, -0.036, 0.979, -0.424 },
				{ 0.000, 2.308, -0.282, 5.852, -0.022, -0.150, -0.060, -0.687, -0.211, 0.059 },
				{ 3.283, -0.282, 26.303, -0.302, -2.874, -0.972, -6.651, -1.910, 1.695, -0.073 },
				{ -0.208, 5.852, -0.302, 43.483, -0.680, -8.601, -1.310, -15.456, -0.289, 6.783 },
				{ -1.030, -0.022, -2.874, -0.680, 3.736, -0.628, 3.470, -0.488, -2.712, 1.162 },
				{ 0.126, -0.150, -0.972, -8.601, -0.628, 5.973, -0.460, 8.282, 1.142, -3.951 },
				{ -1.733, -0.060, -6.651, -1.310, 3.470, -0.460, 7.139, -0.360, -2.703, 0.966 },
				{ -0.036, -0.687, -1.910, -15.456, -0.488, 8.282, -0.360, 14.244, 1.015, -4.777 },
				{ 0.979, -0.211, 1.695, -0.289, -2.712, 1.142, -2.703, 1.015, 3.950, -1.141 },
				{ -0.424, 0.059, -0.073, 6.783, 1.162, -3.951, 0.966, -4.777, -1.141, 5.874 }
			});

			// Create and analyze model, in order to get the global stiffness
			XModel<IXCrackElement> model = CreateModel3x1();
			model.Initialize();
			(IAlgebraicModel algebraicModel, IGlobalVector globalU, IMatrixView globalK) = RunAnalysis(model);

			// Dof numbering
			// node 0: ux_0=con uy_0=con
			// node 1: ux_1=0  uy_1=1  t0x_1=2  t0y_1=3  t1x_1=4  t1y_1=5  t2x_1=6  t2y_1=7  t3x_1=8  t3y_1=9
			// node 2: ux_2=10 uy_2=11 t0x_2=12 t0y_2=13 t1x_2=14 t1y_2=15 t2x_2=16 t2y_2=17 t3x_1=18 t3y_2=19
			// node 3: ux_3=con uy_3=con
			// node 4: ux_4=20 uy_1=21 t0x_4=22 t0y_4=23 t1x_4=24 t1y_4=25 t2x_4=26 t2y_4=27 t3x_1=28 t3y_4=29 
			// node 5: ux_5=30 uy_5=con hx_5=31  hy_5=32
			// node 6: ux_6=33 uy_6=con hx_6=34  hy_6=35
			// node 7: ux_7=36 uy_1=37 t0x_7=38 t0y_7=39 t1x_7=40 t1y_7=41 t2x_7=42 t2y_7=43 t3x_1=44 t3y_7=45 

			// Calculate relevant stiffness submatrix global
			XNode node7 = model.Nodes[7];
			int[] node7GlobalDofs = { 36, 37, 38, 39, 40, 41, 42, 43, 44, 45 };
			IMatrix node7GlobalStiffness = globalK.GetSubmatrix(node7GlobalDofs, node7GlobalDofs);

			// Element 1 dofs (h = heaviside, ti = tip enrichement i):
			// (node 1) ux_1 uy_1 t0x_1 t0y_1 t1x_1 t1y_1 t2x_1 t2y_1 t3x_1 t3y_1 ->   { 0 1 2 3 4 5 6 7 8 9 ...
			// (node 4) ux_4 uy_1 t0x_4 t0y_4 t1x_4 t1y_4 t2x_4 t2y_4 t3x_1 t3y_4 -> ... 20 21 22 23 24 25 26 27 28 29 ...
			// (node 7) ux_7 uy_1 t0x_7 t0y_7 t1x_7 t1y_7 t2x_7 t2y_7 t3x_1 t3y_7 -> ... 36 37 38 39 40 41 42 43 44 45 ...
			// (node 2) ux_2 uy_1 t0x_2 t0y_2 t1x_2 t1y_2 t2x_2 t2y_2 t3x_1 t3y_2 -> ... 10 11 12 13 14 15 16 17 18 19 }

			// Element 2 dofs (h = heaviside, ti = tip enrichement i):
			// (node 4) ux_4 uy_1 t0x_4 t0y_4 t1x_4 t1y_4 t2x_4 t2y_4 t3x_1 t3y_4 ->   { 0 1 2 3 4 5 6 7 8 9 ...
			// (node 5) ux_5 uy_5 hx_5  hy_5                                      -> ... 30 con 31 32 ...
			// (node 6) ux_6 uy_6 hx_6  hy_6                                      -> ... 33 con 34 35 ...
			// (node 7) ux_7 uy_1 t0x_7 t0y_7 t1x_7 t1y_7 t2x_7 t2y_7 t3x_1 t3y_7 -> ... 36 37 38 39 40 41 42 43 44 45 }

			#region delete
			// Element 1 dofs (std first):
			// (N1,ux,0) (N1,uy,1) (N4,ux,2) (N4,uy,3) (N7,ux,4) (N7,uy,5) (N2,ux,6) (N2,uy,7)
			// (N1,tip0x, 8) (N1,tip0y,9) (N1,tip1x,10) (N1,tip1y,11) (N1,tip2x,12) (N1,tip2y,13) (N1,tip3x,14) (N1,tip3y,15)
			// (N4,tip0x,16) (N4,tip0y,17) (N4,tip1x,18) (N4,tip1y,19) (N4,tip2x,20) (N4,tip2y,21) (N4,tip3x,22) (N4,tip3y,23)
			// (N7,tip0x,24) (N7,tip0y,25) (N7,tip1x,26) (N7,tip1y,27) (N7,tip2x,28) (N7,tip2y,29) (N7,tip3x,30) (N7,tip3y,31)
			// (N2,tip0x,32) (N2,tip0y,33) (N2,tip1x,34) (N2,tip1y,35) (N2,tip2x,36) (N2,tip2y,37) (N2,tip3x,38) (N2,tip3y,39)

			// Element 2 dofs (std first):
			// (N4,ux,0) (N4,uy,1) (N5,ux,2) (N5,uy,3) (N6,ux,4) (N6,uy,5) (N7,ux,6) (N7,uy,7)
			// (N4,tip0x,8) (N4,tip0y,9) (N4,tip1x,10) (N4,tip1y,11) (N4,tip2x,12) (N4,tip2y,13) (N4,tip3x,14) (N4,tip3y,15)
			// (N5,bodyX,16) (N5,bodyY,17)
			// (N6,bodyX,18) (N6,bodyY,19)
			// (N7,tip0x,20) (N7,tip0y,21) (N7,tip1x,22) (N7,tip1y,23) (N7,tip2x,24) (N7,tip2y,25) (N7,tip3x,26) (N7,tip3y,27)
			#endregion

			// Calculate relevant stiffness submatrices from elements
			IMatrix elem1Stiffness = model.Elements[1].StiffnessMatrix(model.Elements[1]);
			IMatrix elem2Stiffness = model.Elements[2].StiffnessMatrix(model.Elements[2]);

			int[] elem2Node6Dofs = { 14, 15, 16, 17 };
			IMatrix node6Stiffness = elem2Stiffness.GetSubmatrix(elem2Node6Dofs, elem2Node6Dofs);
			int[] elem1Node7Dofs = { 20, 21, 22, 23, 24, 25, 26, 27, 28, 29 };
			IMatrix node7Elem1Stiffness = elem1Stiffness.GetSubmatrix(elem1Node7Dofs, elem1Node7Dofs);
			int[] elem2Node7Dofs = { 18, 19, 20, 21, 22, 23, 24, 25, 26, 27 };
			IMatrix node7Elem2Stiffness = elem2Stiffness.GetSubmatrix(elem2Node7Dofs, elem2Node7Dofs);

			// Check matrices
			double tol = 1E-13;
			Func<double, double> round = x => 1E6 * Math.Round(x * 1E-6, 3);
			Assert.True(node6StiffnessExpected.Equals(node6Stiffness.DoToAllEntries(round), tol));
			Assert.True(node7Elem1StiffnessExpected.Equals(node7Elem1Stiffness.DoToAllEntries(round), tol));
			Assert.True(node7Elem2StiffnessExpected.Equals(node7Elem2Stiffness.DoToAllEntries(round), tol));
			Assert.True(node7GlobalStiffnessExpected.Equals(node7GlobalStiffness.DoToAllEntries(round), tol));
		}

		[Theory]
		[InlineData(15, 1.0, 2.981, 2559.729)]
		[InlineData(15, 2.0, 2.286, 2241.703)]
		[InlineData(15, 3.0, 2.119, 2158.025)]
		[InlineData(15, 4.0, 2.117, 2157.079)]
		[InlineData(15, 5.0, 2.115, 2156.142)]

		[InlineData(25, 1.0, 2.921, 2533.527)]
		[InlineData(25, 2.0, 2.285, 2240.865)]
		[InlineData(25, 3.0, 2.114, 2155.333)]
		[InlineData(25, 4.0, 2.113, 2154.904)]
		[InlineData(25, 5.0, 2.112, 2154.240)]

		[InlineData(45, 1.0, 2.869, 2510.949)]
		[InlineData(45, 2.0, 2.274, 2235.567)]
		[InlineData(45, 3.0, 2.101, 2148.986)]
		[InlineData(45, 4.0, 2.101, 2148.936)]
		[InlineData(45, 5.0, 2.100, 2148.523)]

		public static void TestJintegral(int numElementsY, double jIntegralRadiusRatio, 
			double expectedJintegral, double expectedSifMode1)
		{
			// Create and analyze model
			int[] numElements = { 3 * numElementsY, numElementsY };
			XModel<IXCrackElement> model = CreateModel(numElements);
			model.Initialize();
			var crack = (Crack)model.GeometryModel.GetDiscontinuity(0);
			(IAlgebraicModel algebraicModel, IGlobalVector globalU, IMatrixView globalK) = RunAnalysis(model);

			// Calculate J-integral and SIFs
			var material = new HomogeneousFractureMaterialField2D(E, v, thickness, false);
			var jIntegrationRule = new JintegrationStrategy(
				GaussLegendre2D.GetQuadratureWithOrder(4, 4),
				new IntegrationWithNonconformingQuads2D(8, GaussLegendre2D.GetQuadratureWithOrder(2, 2)));

			double elementSize = Math.Max(maxCoords[0] / numElements[0], maxCoords[1] / numElements[1]);

			var propagator = new JintegralPropagator2D(model, jIntegralRadiusRatio, jIntegrationRule, material, 
				new MaximumCircumferentialTensileStressCriterion(), 
				new ConstantIncrement2D(1.5 * jIntegralRadiusRatio * elementSize));
			(double growthAngle, double growthLength) = propagator.Propagate(algebraicModel, globalU,
				crack.TipCoordinates, crack.TipSystem, crack.TipElements);

			double sifMode1 = propagator.Logger.SIFsMode1[0];
			double sifMode2 = propagator.Logger.SIFsMode2[0];
			double jIntegral = (Math.Pow(sifMode1, 2) + Math.Pow(sifMode2, 2)) / material.EquivalentYoungModulus;

			// Check. Note that I allow better values than the reference solution, since my implementation could converge faster.
			Assert.InRange(Math.Round(jIntegral, 3), 2.100, expectedJintegral);
			Assert.InRange(Math.Round(sifMode1, 3), 2148.523, expectedSifMode1);
		}

		private static void ApplyBoundaryConditions(XModel<IXCrackElement> model)
		{
			// Boundary conditions
			double tol = 1E-6;
			double L = maxCoords[0];
			double H = maxCoords[1];
			XNode topRight = model.Nodes.Values.Where(n => Math.Abs(n.X - L) <= tol && Math.Abs(n.Y - H) <= tol).First();
			XNode bottomRight = model.Nodes.Values.Where(n => Math.Abs(n.X - L) <= tol && Math.Abs(n.Y) <= tol).First();
			topRight.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = +0.05 });
			bottomRight.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = -0.05 });
			foreach (XNode node in model.Nodes.Values.Where(n => Math.Abs(n.X) <= tol))
			{
				node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0.0 });
				node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0.0 });
			}
		}

		private static XModel<IXCrackElement> CreateModel3x1()
		{
			var model = new XModel<IXCrackElement>(2);
			model.Subdomains[subdomainID] = new XSubdomain<IXCrackElement>(subdomainID);

			// Nodes
			var nodes = new XNode[8];
			nodes[0] = new XNode(0, new double[] { 0.0, 0.0 });
			nodes[1] = new XNode(1, new double[] { 1.0/3 * maxCoords[0], 0.0 });
			nodes[2] = new XNode(2, new double[] { 1.0/3 * maxCoords[0], maxCoords[1] });
			nodes[3] = new XNode(3, new double[] { 0.0, maxCoords[1] });
			nodes[4] = new XNode(4, new double[] { 2.0/3 * maxCoords[0], 0.0 });
			nodes[5] = new XNode(5, new double[] { maxCoords[0], 0.0 });
			nodes[6] = new XNode(6, new double[] { maxCoords[0], maxCoords[1] });
			nodes[7] = new XNode(7, new double[] { 2.0 / 3 * maxCoords[0], maxCoords[1] });

			foreach (XNode node in nodes)
			{
				model.Nodes[node.ID] = node;
			}

			// Elements
			var connectivity = new int[3, 4]
			{
				{ 0, 1, 2, 3 },
				{ 1, 4, 7, 2 },
				{ 4, 5, 6, 7 }
			};
			var material = new HomogeneousFractureMaterialField2D(E, v, thickness, false);
			var enrichedIntegration = new IntegrationWithNonconformingQuads2D(8, GaussLegendre2D.GetQuadratureWithOrder(2, 2));
			var bulkIntegration = new CrackElementIntegrationStrategy(
				enrichedIntegration, enrichedIntegration, enrichedIntegration);
			var factory = new XCrackElementFactory2D(material, thickness, bulkIntegration);
			for (int e = 0; e < 3; ++e)
			{
				var elementNodes = new XNode[4];
				for (int n = 0; n < 4; ++n)
				{
					elementNodes[n] = model.Nodes[connectivity[e, n]];
				}
				IXCrackElement element = factory.CreateElement(e, CellType.Quad4, elementNodes);
				model.Elements[element.ID] = element;
				model.Subdomains[subdomainID].Elements.Add(element);
			}

			ApplyBoundaryConditions(model);

			// Crack, enrichments
			var geometryModel = new CrackGeometryModel(model);
			model.GeometryModel = geometryModel;
			geometryModel.Enricher = new NodeEnricherIndependentCracks(geometryModel, new NullSingularityResolver());
			var crack = new Crack(model);
			geometryModel.Cracks[crack.ID] = crack;

			return model;
		}

		private static XModel<IXCrackElement> CreateModel(int[] numElements)
		{
			var model = new XModel<IXCrackElement>(2);
			model.Subdomains[subdomainID] = new XSubdomain<IXCrackElement>(subdomainID);

			// Materials, integration
			var material = new HomogeneousFractureMaterialField2D(E, v, thickness, false);
			var enrichedIntegration = new IntegrationWithNonconformingQuads2D(8, GaussLegendre2D.GetQuadratureWithOrder(2, 2));
			var bulkIntegration = new CrackElementIntegrationStrategy(
				enrichedIntegration, enrichedIntegration, enrichedIntegration);
			var factory = new XCrackElementFactory2D(material, thickness, bulkIntegration);

			// Mesh
			var mesh = new UniformCartesianMesh2D.Builder(minCoords, maxCoords, numElements).BuildMesh();
			Utilities.Models.AddNodesElements(model, mesh, factory);

			ApplyBoundaryConditions(model);

			// Crack, enrichments
			var geometryModel = new CrackGeometryModel(model);
			model.GeometryModel = geometryModel;
			geometryModel.Enricher = new NodeEnricherIndependentCracks(geometryModel, new NullSingularityResolver());
			var crack = new Crack(model);
			geometryModel.Cracks[crack.ID] = crack;

			return model;
		}

		private static (IAlgebraicModel algebraicModel, IGlobalVector globalU, IMatrixView globalK) 
			RunAnalysis(XModel<IXCrackElement> model)
		{
			// Solver
			var factory = new SkylineSolver.Factory();
			GlobalAlgebraicModel<SkylineMatrix> algebraicModel = factory.BuildAlgebraicModel(model);
			var solver = factory.BuildSolver(algebraicModel);
			solver.PreventFromOverwrittingSystemMatrices(); // Necessary to extract the stiffness matrix.

			// Problem type
			var provider = new ProblemStructural(model, algebraicModel, solver);

			// Analyzers
			var childAnalyzer = new LinearAnalyzer(model, algebraicModel, solver, provider);
			var parentAnalyzer = new StaticAnalyzer(model, algebraicModel, solver, provider, childAnalyzer);

			// Run the anlaysis 
			parentAnalyzer.Initialize();
			parentAnalyzer.Solve();

			return (algebraicModel, algebraicModel.LinearSystem.Solution, algebraicModel.LinearSystem.Matrix.SingleMatrix);
		}

		private class Crack : ICrack
		{
			private readonly ExteriorLsmCrack2D crack;
			private readonly double[] mouth, tip;
			private readonly XModel<IXCrackElement> model;

			public Crack(XModel<IXCrackElement> model)
			{
				this.mouth = new double[] { maxCoords[0], 0.5 * maxCoords[1] };
				this.tip = new double[] { 0.5 * maxCoords[0], 0.5 * maxCoords[1] };
				var initialGeom = new PolyLine2D(mouth, tip);
				this.crack = new ExteriorLsmCrack2D(0, initialGeom, model, null);
				this.model = model;
			}

			public HashSet<IXCrackElement> ConformingElements => crack.ConformingElements;

			public EnrichmentItem CrackBodyEnrichment { get; private set; }

			public IXGeometryDescription CrackGeometry => crack.CrackGeometry;

			public EnrichmentItem CrackTipEnrichments => crack.CrackTipEnrichments;

			public HashSet<IXCrackElement> IntersectedElements => crack.IntersectedElements;

			public double[] TipCoordinates => crack.TipCoordinates;

			public HashSet<IXCrackElement> TipElements => crack.TipElements;

			public TipCoordinateSystem TipSystem => crack.TipSystem;

			public int ID => crack.ID;

			public int MouthElementID { get; set; }

			public void CheckPropagation(IPropagationTermination termination) => crack.CheckPropagation(termination);

			public IList<EnrichmentItem> DefineEnrichments(int numCurrentEnrichments)
			{
				IList<EnrichmentItem> enrichments = crack.DefineEnrichments(numCurrentEnrichments);

				// Different crack body enrichment
				var stepEnrichmentFunc = new OppositeStepEnrichment(this);
				IDofType[] stepEnrichedDofs =
				{
				new EnrichedDof(stepEnrichmentFunc, StructuralDof.TranslationX),
				new EnrichedDof(stepEnrichmentFunc, StructuralDof.TranslationY)
				};
				this.CrackBodyEnrichment = new EnrichmentItem(
					crack.CrackBodyEnrichment.ID, new IEnrichmentFunction[] { stepEnrichmentFunc }, stepEnrichedDofs);

				enrichments[0] = this.CrackBodyEnrichment;
				return enrichments;
			}

			public HashSet<XNode> FindNodesNearFront(double maxDistance)
			{
				var circle = new Circle2D(TipCoordinates, maxDistance); //TODO: This needs adapting for 3D
				return MeshUtilities.FindNodesInsideCircle(circle, TipElements.First());
			}

			public void InitializeGeometry()
			{
				crack.InitializeGeometry();
			}

			public void InteractWithMesh()
			{
				crack.InteractWithMesh();
				foreach (XCrackElement2D element in model.Elements.Values)
				{
					if (element.InteractingCracks.ContainsKey(crack))
					{
						element.InteractingCracks[this] = element.InteractingCracks[crack];
						element.InteractingCracks.Remove(crack);
					}
					if (IsMouthElement(element.Nodes))
					{
						MouthElementID = element.ID;
					}
				}
			}

			public void UpdateGeometry(IAlgebraicModel algebraicModel, IGlobalVector totalDisplacements)
			{
				crack.UpdateGeometry(algebraicModel, totalDisplacements);
			}

			private bool IsMouthElement(IReadOnlyList<XNode> nodes)
			{
				double tol = 1E-6;
				bool result = (nodes[0].Coordinates[0] < mouth[0]) && (nodes[0].Coordinates[1] < mouth[1]);
				result &= (Math.Abs(nodes[1].Coordinates[0] - mouth[0]) <= tol) && (nodes[1].Coordinates[1] < mouth[1]);
				result &= (Math.Abs(nodes[2].Coordinates[0] - mouth[0]) <= tol) && (nodes[2].Coordinates[1] > mouth[1]);
				result &= (nodes[3].Coordinates[0] < mouth[0]) && (nodes[3].Coordinates[1] > mouth[1]);
				return result;
			}
		}

		private class OppositeStepEnrichment : IEnrichmentFunction
		{
			private CrackStepEnrichment enrichment;

			public OppositeStepEnrichment(ICrack crack)
			{
				this.enrichment = new CrackStepEnrichment(crack);
			}

			public EvaluatedFunction EvaluateAllAt(XPoint point)
			{
				var oppositeResult = enrichment.EvaluateAllAt(point);
				return new EvaluatedFunction(-oppositeResult.Value, oppositeResult.CartesianDerivatives);
			}

			public double EvaluateAt(XNode node) => -enrichment.EvaluateAt(node);

			public double EvaluateAt(XPoint point) => -enrichment.EvaluateAt(point);

			public double EvaluateJumpAcross(IXDiscontinuity discontinuity, XPoint point)
				=> enrichment.EvaluateJumpAcross(discontinuity, point);
		}
	}
}
