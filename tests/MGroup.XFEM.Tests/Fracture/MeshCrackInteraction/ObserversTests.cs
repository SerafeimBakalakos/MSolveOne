using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.MSolve.Meshes.Structured;
using MGroup.XFEM.Cracks;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Enrichment.Enrichers;
using MGroup.XFEM.Enrichment.Observers;
using MGroup.XFEM.Enrichment.SingularityResolution;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Integration;
using MGroup.XFEM.Integration.Quadratures;
using MGroup.XFEM.Materials;
using MGroup.XFEM.Output.Mesh;
using MGroup.XFEM.Output.Vtk;
using MGroup.XFEM.Output.Writers;
using Xunit;

namespace MGroup.XFEM.Tests.Fracture.Observers
{
	public static class ObserversTests
	{
		private const int subdomainID = 0;

		[Fact]
		public static void TestCrackElementInteraction()
		{
			XModel<IXCrackElement> model = CreateModel();
			var crack = (ExteriorLsmCrack)model.GeometryModel.GetDiscontinuity(0);

			// Expected results
			var expectedTipElements = new List<int[]>();
			expectedTipElements.Add(new int[] { 184 });
			expectedTipElements.Add(new int[] { 205 });
			expectedTipElements.Add(new int[] { 226 });
			expectedTipElements.Add(new int[] { 270 });

			var expectedIntersectedElements = new List<int[]>();
			expectedIntersectedElements.Add(new int[] { 180, 181, 182, 183 });
			expectedIntersectedElements.Add(new int[] { 180, 181, 182, 183, 184, 204 });
			expectedIntersectedElements.Add(new int[] { 180, 181, 182, 183, 184, 204, 205, 206 });
			expectedIntersectedElements.Add(new int[] { 180, 181, 182, 183, 184, 204, 205, 206, 226, 246, 247, 248, 268, 269 });

			var expectedConformingElements = new List<int[]>();
			expectedConformingElements.Add(new int[0]);
			expectedConformingElements.Add(new int[0]);
			expectedConformingElements.Add(new int[0]);
			expectedConformingElements.Add(new int[0]);

			// Compare the model's state at each iteration.
			for (int t = 0; t <= 3; ++t)
			{
				if (t == 0) model.Initialize();
				else model.Update(null, null);

				// Check
				var computedTipElements = new HashSet<int>(crack.TipElements.Select(e => e.ID));
				var computedIntersectedElements = new HashSet<int>(crack.IntersectedElements.Select(e => e.ID));
				var computedConformingElements = new HashSet<int>(crack.ConformingElements.Select(e => e.ID));
				Assert.True(computedTipElements.SetEquals(expectedTipElements[t]));
				Assert.True(computedIntersectedElements.SetEquals(expectedIntersectedElements[t]));
				Assert.True(computedConformingElements.SetEquals(expectedConformingElements[t]));
			}
		}

		[Fact]
		public static void TestEnrichedNodes()
		{
			XModel<IXCrackElement> model = CreateModel();
			var crack = (ExteriorLsmCrack)model.GeometryModel.GetDiscontinuity(0);

			// Set up observers.
			var newTipNodes = new NewCrackTipNodesObserver();
			var previousTipNodes = new PreviousCrackTipNodesObserver();
			var allBodyNodes = new AllCrackStepNodesObserver();
			var newBodyNodes = new NewCrackStepNodesObserver();
			var rejectedBodyNodes = new RejectedCrackStepNodesObserver(crack, newTipNodes, allBodyNodes);
			var bodyNodesWithModifiedLevelSet = new CrackStepNodesWithModifiedLevelSetObserver(crack);
			var modifiedNodes = new NodesWithModifiedEnrichmentsObserver(bodyNodesWithModifiedLevelSet);
			var modifiedElements = new ElementsWithModifiedNodesObserver(modifiedNodes);
			var nearModifiedNodes = new NodesNearModifiedNodesObserver(modifiedNodes, modifiedElements);
			var compositeObserver = new CompositeEnrichmentObserver();
			compositeObserver.AddObservers(newTipNodes, previousTipNodes, allBodyNodes, newBodyNodes, rejectedBodyNodes,
				bodyNodesWithModifiedLevelSet, modifiedNodes, modifiedElements, nearModifiedNodes);
			model.GeometryModel.Enricher.Observers.Add(compositeObserver);

			// Expected results
			var expectedNewTipNodes = new List<int[]>();
			expectedNewTipNodes.Add(new int[] { 193, 194, 215, 214 });
			expectedNewTipNodes.Add(new int[] { 215, 216, 237, 236 });
			expectedNewTipNodes.Add(new int[] { 237, 238, 259, 258 });
			expectedNewTipNodes.Add(new int[] { 283, 284, 305, 304 });

			var expectedOldTipNodes = new List<int[]>();
			expectedOldTipNodes.Add(new int[] {  });
			expectedOldTipNodes.Add(new int[] { 193, 194, 215, 214 });
			expectedOldTipNodes.Add(new int[] { 215, 216, 237, 236 });
			expectedOldTipNodes.Add(new int[] { 237, 238, 259, 258 });

			var expectedBodyNodes = new List<int[]>();
			expectedBodyNodes.Add(new int[] { 211, 210, 212, 213 });
			expectedBodyNodes.Add(new int[] { 211, 210, 212, 213, 214 });
			expectedBodyNodes.Add(new int[] { 211, 210, 212, 213, 214, 215, 236, 216 });
			expectedBodyNodes.Add(new int[] 
			{ 
				211, 210, 212, 213, 214, 215, 236, 216, 237, 238, 259, 258, 280, 260, 281, 261, 282, 303, 302 
			});

			var expectedNewBodyNodes = new List<int[]>();
			expectedNewBodyNodes.Add(new int[] { 211, 210, 212, 213 });
			expectedNewBodyNodes.Add(new int[] { 214 });
			expectedNewBodyNodes.Add(new int[] { 215, 236, 216 });
			expectedNewBodyNodes.Add(new int[] { 237, 238, 259, 258, 280, 260, 281, 261, 282, 303, 302 });

			var expectedRejectedBodyNodes = new List<int[]>();
			expectedRejectedBodyNodes.Add(new int[] { 189, 190, 191, 192 });
			expectedRejectedBodyNodes.Add(new int[] { 189, 190, 191, 192, 193, 194, 235 });
			expectedRejectedBodyNodes.Add(new int[] { 189, 190, 191, 192, 193, 194, 235, 217 });
			expectedRejectedBodyNodes.Add(new int[] { 189, 190, 191, 192, 193, 194, 235, 217, 279 });

			var expectedNearModifiedBodyNodes = new List<int[]>();
			expectedNearModifiedBodyNodes.Add(new int[] { });
			expectedNearModifiedBodyNodes.Add(new int[] { 213 });
			expectedNearModifiedBodyNodes.Add(new int[] { 214 });
			expectedNearModifiedBodyNodes.Add(new int[] { 215, 216, 236 });

			// Compare the model's state at each iteration.
			for (int t = 0; t <= 3; ++t)
			{
				if (t == 0) model.Initialize();
				else model.Update(null, null);

				// Check
				var computedNewTipNodes = new HashSet<int>(newTipNodes.NewCrackTipNodes.Select(n => n.ID));
				var computedOldTipNodes = new HashSet<int>(previousTipNodes.PreviousCrackTipNodes.Select(n => n.ID));
				var computedBodyNodes = new HashSet<int>(allBodyNodes.AllCrackStepNodes.Select(n => n.ID));
				var computedNewBodyNodes = new HashSet<int>(newBodyNodes.NewCrackStepNodes.Select(n => n.ID));
				var computedRejectedBodyNodes = new HashSet<int>(rejectedBodyNodes.RejectedHeavisideNodes.Select(n => n.ID));
				var computedNearModifiedNodes = new HashSet<int>(nearModifiedNodes.NearModifiedNodes.Select(n => n.ID));

				Assert.True(computedNewTipNodes.SetEquals(expectedNewTipNodes[t]));
				Assert.True(computedOldTipNodes.SetEquals(expectedOldTipNodes[t]));
				Assert.True(computedBodyNodes.SetEquals(expectedBodyNodes[t]));
				Assert.True(computedNewBodyNodes.SetEquals(expectedNewBodyNodes[t]));
				Assert.True(computedRejectedBodyNodes.SetEquals(expectedRejectedBodyNodes[t]));
				Assert.True(computedNearModifiedNodes.SetEquals(expectedNearModifiedBodyNodes[t]));
			}
		}

		//[Fact]
		private static void RunPropagationAndPlot() 
		{
			XModel<IXCrackElement> model = CreateModel();
			var crack = (ExteriorLsmCrack)model.GeometryModel.GetDiscontinuity(0);

			// Plot the FE mesh
			string outputDir = @"C:\Users\Serafeim\Desktop\XFEM2020\Cracks\InteractionTests\";
			var outputMesh = new ContinuousOutputMesh(model.Nodes.Values.OrderBy(n => n.ID).ToList(), model.EnumerateElements());
			string meshPath = outputDir + "mesh.vtk";
			using (var writer = new VtkFileWriter(meshPath))
			{
				writer.WriteMesh(outputMesh);
			}

			// Set up observers. //TODO: Each observer should define and link all its necessary observers in an optional constructor.
			crack.Observers.Add(new CrackLevelSetPlotter(crack, outputMesh, outputDir));
			crack.Observers.Add(new CrackInteractingElementsPlotter(crack, outputDir));

			var newTipNodes = new NewCrackTipNodesObserver();
			var previousTipNodes = new PreviousCrackTipNodesObserver();
			var allBodyNodes = new AllCrackStepNodesObserver();
			var newBodyNodes = new NewCrackStepNodesObserver();
			var rejectedBodyNodes = new RejectedCrackStepNodesObserver(crack, newTipNodes, allBodyNodes);
			var bodyNodesWithModifiedLevelSet = new CrackStepNodesWithModifiedLevelSetObserver(crack);
			var modifiedNodes = new NodesWithModifiedEnrichmentsObserver(bodyNodesWithModifiedLevelSet);
			var modifiedElements = new ElementsWithModifiedNodesObserver(modifiedNodes);
			var nearModifiedNodes = new NodesNearModifiedNodesObserver(modifiedNodes, modifiedElements);
			var compositeObserver = new CompositeEnrichmentObserver();
			var enrichmentPlotter = new CrackEnrichmentPlotter(crack, outputDir, newTipNodes, previousTipNodes, allBodyNodes,
				newBodyNodes, rejectedBodyNodes, nearModifiedNodes);
			compositeObserver.AddObservers(newTipNodes, previousTipNodes, allBodyNodes, newBodyNodes, rejectedBodyNodes,
				bodyNodesWithModifiedLevelSet, modifiedNodes, modifiedElements, nearModifiedNodes);
			model.GeometryModel.Enricher.Observers.Add(compositeObserver);

			// Propagate the crack. During this the observers will plot the data they pull from model. 
			model.Initialize();
			for (int rep = 0; rep < 3; ++rep)
			{
				model.Update(null, null);
			}
		}

		private static XModel<IXCrackElement> CreateModel()
		{
			var model = new XModel<IXCrackElement>(2);
			model.Subdomains[subdomainID] = new XSubdomain<IXCrackElement>(subdomainID);
			model.FindConformingSubcells = true;

			// Materials, integration
			double E = 2E6, v = 0.3, thickness = 1.0;
			var material = new HomogeneousFractureMaterialField2D(E, v, thickness, true);
			var enrichedIntegration = new IntegrationWithNonconformingQuads2D(8, GaussLegendre2D.GetQuadratureWithOrder(2, 2));
			var bulkIntegration = new CrackElementIntegrationStrategy(
				enrichedIntegration, enrichedIntegration, enrichedIntegration);
			var factory = new XCrackElementFactory2D(material, thickness, bulkIntegration);

			// Mesh
			double[] minCoords = { 0.0, 0.0 };
			double[] maxCoords = { 20.0, 20.0 };
			int[] numElements = { 20, 20 };
			var mesh = new UniformCartesianMesh2D.Builder(minCoords, maxCoords, numElements).BuildMesh();
			Utilities.Models.AddNodesElements(model, mesh, factory);

			// Fixed crack path
			double[] angles = { Math.PI/6, Math.PI / 6, -Math.PI / 6 };
			double[] lengths = { 1.0, 1.5, 4.0 };
			IPropagator propagator = new MockPropagator(angles, lengths);

			// Crack, enrichments
			var geometryModel = new CrackGeometryModel(model);
			model.GeometryModel = geometryModel;
			geometryModel.Enricher = new NodeEnricherIndependentCracks(geometryModel, new RelativeAreaSingularityResolver(0.006));
			double yCrack = 9.99; // avoid conforming case
			var initialFlaw = new PolyLine2D(new double[] { 0, yCrack }, new double[] { 4.93, yCrack } );
			var crack = new ExteriorLsmCrack(0, initialFlaw, model, propagator);
			geometryModel.Cracks[crack.ID] = crack;

			return model;
		}
	}
}
