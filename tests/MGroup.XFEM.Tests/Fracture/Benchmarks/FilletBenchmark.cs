using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Loads;
using MGroup.MSolve.Discretization.Mesh;
using MGroup.MSolve.Meshes.Unstructured;
using MGroup.MSolve.Meshes.Unstructured.Gmsh;
using MGroup.Solvers.AlgebraicModel;
using MGroup.Solvers.Direct;
using MGroup.XFEM.Analysis;
using MGroup.XFEM.Cracks;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Cracks.Jintegral;
using MGroup.XFEM.Cracks.PropagationCriteria;
using MGroup.XFEM.Cracks.PropagationTermination;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment.Enrichers;
using MGroup.XFEM.Enrichment.SingularityResolution;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Boundaries;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Integration;
using MGroup.XFEM.Integration.Quadratures;
using MGroup.XFEM.Materials;
using Xunit;

namespace MGroup.XFEM.Tests.Fracture.Benchmarks
{
	public static class FilletBenchmark
	{
		/// <summary>
		/// Thickness of the whole domain
		/// </summary>
		private const double t = 1.0; // mm

		/// <summary>
		/// Young's modulus
		/// </summary>
		private const double E = 2.1E12; // kN/mm^2

		/// <summary>
		/// Poisson's ratio
		/// </summary>
		private const double v = 0.3;

		private const bool planeStress = false;

		/// <summary>
		/// The maximum value that the effective SIF can reach before collapse occurs.
		/// </summary>
		private const double fractureToughness = double.MaxValue;

		/// <summary>
		/// Magnitude in kN of "opening" load
		/// </summary>
		private const double load = 20.0; // kN

		// Geometry (m)
		private const double bottomWidth = 375.0, topWidth = 75.0, radius = 20.0;// mm
		private const double flangeHeight = 75.0, totalHeight = 150.0; // mm
		private const double crackHeight = flangeHeight + radius, crackLength = 5, webLeft = 0.5 * (bottomWidth - topWidth); //mm
		private const double infCrackHeight = 90.0, supCrackHeight = 105.0; //mm

		// Mesh
		private static readonly string meshPath = Path.Combine(new string[] 
		{ 
			Directory.GetParent(Directory.GetCurrentDirectory()).Parent.FullName, "Resources", "fillet_1272dofs.msh" 
		});

		private const double growthLength = 5; // mm. Must be sufficiently larger than the element size.

		// Usually should be in [1.5, 2.5). The J-integral radius must be large enough to at least include elements around
		// the element that contains the crack tip. However it must not be so large that an element intersected by the 
		// J-integral contour is containes the previous crack tip. Thus the J-integral radius must be sufficiently smaller
		// than the crack growth length.
		private const double jIntegralRadiusRatio = 2.0;

		private const double heavisideTol = 1E-2;
		private const double tipEnrichmentArea = 0.0;
		private const int maxIterations = 13;
		private const int subdomainID = 0;

		[Theory]
		[InlineData(true)]
		[InlineData(false)]
		public static void TestCrackPropagationPath(bool rigidBCs)
		{
			XModel<IXCrackElement> model = CreateModel(rigidBCs);
			RunAnalysis(model);
			var crack = (ExteriorLsmCrack)model.GeometryModel.GetDiscontinuity(0);

			// Expected propagation path
			var expectedPath = new List<double[]>();
			if (rigidBCs)
			{
				expectedPath.Add(new double[] { 150.000000000000, 95.0000000000000 });
				expectedPath.Add(new double[] { 155.000000000000, 95.0000000000000 });
				expectedPath.Add(new double[] { 159.928845175266, 94.1594732376293 });
				expectedPath.Add(new double[] { 164.895272288676, 93.5810255501270 });
				expectedPath.Add(new double[] { 169.873369876570, 93.1135382183093 });
				expectedPath.Add(new double[] { 174.865946489105, 92.8411804242900 });
				expectedPath.Add(new double[] { 179.863172733028, 92.6746575518206 });
				expectedPath.Add(new double[] { 184.863046509768, 92.6391298631669 });
				expectedPath.Add(new double[] { 189.862887217122, 92.6790410281461 });
				expectedPath.Add(new double[] { 194.861153856214, 92.8106867821075 });
				expectedPath.Add(new double[] { 199.854887645723, 93.0609323405507 });
				expectedPath.Add(new double[] { 204.846200988297, 93.3555356607026 });
				expectedPath.Add(new double[] { 209.835693764484, 93.6795138025205 });
				expectedPath.Add(new double[] { 214.822110231868, 94.0478218445237 });
				//expectedPath.Add(new double[] { 219.810510335299, 94.3882104578577 });
			}
			else
			{
				expectedPath.Add(new double[] { 150.000000000000, 95.0000000000000 });
				expectedPath.Add(new double[] { 155.000000000000, 95.0000000000000 });
				expectedPath.Add(new double[] { 159.546278695233, 92.9188104302414 });
				expectedPath.Add(new double[] { 163.855055966478, 90.3821909422607 });
				expectedPath.Add(new double[] { 167.842709039709, 87.3657985660029 });
				expectedPath.Add(new double[] { 171.568844458286, 84.0317447508364 });
				expectedPath.Add(new double[] { 175.075157758010, 80.4672297706211 });
				expectedPath.Add(new double[] { 178.428998912345, 76.7588946304959 });
				expectedPath.Add(new double[] { 181.635027717824, 72.9220496638023 });
				expectedPath.Add(new double[] { 184.762601328988, 69.0209878459438 });
				expectedPath.Add(new double[] { 187.806841672205, 65.0545535867224 });
				expectedPath.Add(new double[] { 190.768299071507, 61.0259347077364 });
				expectedPath.Add(new double[] { 193.591488900770, 56.8992412143998 });
				expectedPath.Add(new double[] { 196.150616188081, 52.6037939120413 });
			}
			
			// Chack propagation path
			Assert.Equal(expectedPath.Count, crack.CrackPath.Count);
			int precision = 3;
			for (int i = 0; i < expectedPath.Count; ++i)
			{
				Assert.Equal(expectedPath[i][0], crack.CrackPath[i][0], precision);
				Assert.Equal(expectedPath[i][1], crack.CrackPath[i][1], precision);
			}
		}

		private static void ApplyBoundaryConditions(XModel<IXCrackElement> model, bool rigidBCs)
		{
			double meshTol = 1E-6;
			// Constraints
			if (rigidBCs)
			{
				foreach (var node in model.Nodes.Values.Where(n => Math.Abs(n.Y) <= meshTol))
				{
					node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0.0 });
					node.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0.0 });
				}
			}
			else // flexible
			{
				XNode bottomLeftNode = model.Nodes.Values.Where(n => (Math.Abs(n.X) <= meshTol) && (Math.Abs(n.Y) <= meshTol)).First();
				XNode bottomRightNode = model.Nodes.Values.Where(
					n => (Math.Abs(n.X - bottomWidth) <= meshTol) && (Math.Abs(n.Y) <= meshTol)).First();
				bottomLeftNode.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0.0 });
				bottomLeftNode.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0.0 });
				bottomRightNode.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationX, Amount = 0.0 });
				bottomRightNode.Constraints.Add(new Constraint() { DOF = StructuralDof.TranslationY, Amount = 0.0 });
			}

			// Distribute load amongst top nodes uniformly
			XNode[] topNodes = model.Nodes.Values.Where(n => Math.Abs(n.Y - totalHeight) <= meshTol).ToArray();
			double distributedLoad = load / topNodes.Length;
			for (int i = 0; i < topNodes.Length; ++i)
			{
				model.NodalLoads.Add(new Load() { Node = topNodes[i], DOF = StructuralDof.TranslationY, Amount = distributedLoad });
			}
		}

		private static XModel<IXCrackElement> CreateModel(bool rigidBCs)
		{
			var model = new XModel<IXCrackElement>(2);
			model.Subdomains[subdomainID] = new XSubdomain(subdomainID);
			model.FindConformingSubcells = true;

			// Mesh generation
			UnstructuredMesh mesh;
			using (var reader = new GmshReader(2, meshPath))
			{
				mesh = reader.CreateMesh();
			}

			// Nodes
			foreach ((int nodeID, double[] coords) in mesh.EnumerateNodes())
			{
				model.Nodes[nodeID] = new XNode(nodeID, coords);
			}

			// Elements
			var material = new HomogeneousFractureMaterialField2D(E, v, t, planeStress);
			var enrichedIntegration = new IntegrationWithNonconformingQuads2D(8, GaussLegendre2D.GetQuadratureWithOrder(2, 2));
			var bulkIntegration = new CrackElementIntegrationStrategy(
				enrichedIntegration, enrichedIntegration, enrichedIntegration);
			var elemFactory = new XCrackElementFactory2D(material, t, bulkIntegration);

			// Elements
			foreach ((int elementID, CellType cellType, int[] connectivity) in mesh.EnumerateElements())
			{
				var nodes = new XNode[connectivity.Length];
				for (int n = 0; n < connectivity.Length; ++n)
				{
					nodes[n] = model.Nodes[connectivity[n]];
				}
				var element = elemFactory.CreateElement(elementID, cellType, nodes);
				model.Elements[elementID] = element;
				model.Subdomains[0].Elements.Add(element);
			}

			ApplyBoundaryConditions(model, rigidBCs);

			// Crack, enrichments
			var geometryModel = new CrackGeometryModel(model);
			model.GeometryModel = geometryModel;
			geometryModel.Enricher = new NodeEnricherIndependentCracks(
				geometryModel, new RelativeAreaSingularityResolver(heavisideTol), tipEnrichmentArea);
			var crackMouth = new double[] { webLeft, crackHeight };
			var crackTip = new double[] { webLeft + crackLength, crackHeight };
			var initialCrack = new PolyLine2D(crackMouth, crackTip);

			////TODO: This is probably better, but my expected results are from the next one
			//var jIntegrationRule = new JintegrationStrategy(
			//    GaussLegendre2D.GetQuadratureWithOrder(4, 4),
			//    new IntegrationWithNonconformingQuads2D(8, GaussLegendre2D.GetQuadratureWithOrder(4, 4)));
			var jIntegrationRule = new IntegrationWithNonconformingQuads2D(8, GaussLegendre2D.GetQuadratureWithOrder(4, 4));
			var propagator = new JintegralPropagator2D(jIntegralRadiusRatio, jIntegrationRule, material,
				new MaximumCircumferentialTensileStressCriterion(), new ConstantIncrement2D(growthLength));
			var crack = new ExteriorLsmCrack(0, initialCrack, model, propagator);
			geometryModel.Cracks[crack.ID] = crack;

			return model;
		}

		private static void RunAnalysis(XModel<IXCrackElement> model)
		{
			// Solver
			var factory = new SkylineSolver.Factory();
			GlobalAlgebraicModel<SkylineMatrix> algebraicModel = factory.BuildAlgebraicModel(model);
			var solver = factory.BuildSolver(algebraicModel);

			var domainBoundary = new FilletBoundary();
			var termination = new TerminationLogic.Or(
				new FractureToughnessTermination(fractureToughness),
				new CrackExitsDomainTermination(domainBoundary));
			var analyzer = new QuasiStaticLefmAnalyzer(model, algebraicModel, solver, maxIterations, termination);

			analyzer.Analyze();
		}

		private class FilletBoundary : IDomainBoundary
		{
			private readonly double voidRectWidth, centerY, leftCenterX, rightCenterX;

			public FilletBoundary()
			{
				voidRectWidth = 0.5 * (bottomWidth - topWidth);
				centerY = flangeHeight + radius;
				leftCenterX = voidRectWidth - radius;
				rightCenterX = bottomWidth - voidRectWidth + radius;
			}

			public bool SurroundsPoint(double[] point)
			{
				// Shapes
				var rectHull = new RectangularDomainBoundary(
					new double[] { 0.0, 0.0 }, new double[] { bottomWidth, totalHeight });
				var leftVoid = new RectangularDomainBoundary(
					new double[] { 0.0, flangeHeight }, new double[] { voidRectWidth, totalHeight });
				var rightVoid = new RectangularDomainBoundary(
					new double[] { bottomWidth - voidRectWidth, flangeHeight },
					new double[] { bottomWidth, totalHeight });
				var leftCircle = new Circle2D(leftCenterX, centerY, radius);
				var rightCircle = new Circle2D(rightCenterX, centerY, radius);

				if (rectHull.SurroundsPoint(point))
				{
					if (leftVoid.SurroundsPoint(point)) // Over flange, left of web
					{
						if ((point[0] > leftCenterX) && (point[1] < centerY))
						{
							if (leftCircle.SignedDistanceOf(point) > 0 )
							{
								return true; // Inside left fillet
							}
							else return false;
						}
						else return false;
					}
					else if (rightVoid.SurroundsPoint(point)) // Over flange, right of web
					{
						if ((point[0] < leftCenterX) && (point[1] < centerY))
						{
							if (rightCircle.SignedDistanceOf(point) > 0)
							{
								return true; // Inside right fillet
							}
							else return false;
						}
						else return false;
					}
					else return true; // Inside the flange or the web
				}
				else return false;
			}
		}
	}
}
