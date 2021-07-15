using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.Environments;
using MGroup.Environments.Mpi;
using MGroup.LinearAlgebra.Distributed.Overlapping;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.DDM.PSM.Dofs;
using MGroup.Solvers.DDM.Tests.ExampleModels;
using MGroup.Solvers.DofOrdering;
using MGroup.Solvers.DofOrdering.Reordering;
using Xunit;

namespace MGroup.Solvers.DDM.Tests.PSM
{
	public class PsmDofManagerTests
	{
		[Theory]
		[InlineData(EnvironmentChoice.SequentialSharedEnvironment)]
		[InlineData(EnvironmentChoice.TplSharedEnvironment)]
		public static void TestForLine1D(EnvironmentChoice environmentChoice) 
			=> TestForLine1DInternal(Utilities.CreateEnvironment(environmentChoice));

		internal static void TestForLine1DInternal(IComputeEnvironment environment)
		{
			ComputeNodeTopology nodeTopology = Line1DExample.CreateNodeTopology();
			environment.Initialize(nodeTopology);

			IModel model = Line1DExample.CreateMultiSubdomainModel();
			model.ConnectDataStructures();
			var subdomainTopology = new SubdomainTopology(environment, model);

			IGlobalFreeDofOrdering dofOrdering = ModelUtilities.OrderDofs(model);
			var dofManager = new PsmDofManager(environment, model, subdomainTopology, 
				s => dofOrdering.SubdomainDofOrderings[s], true);
			environment.DoPerNode(s => dofManager.GetSubdomainDofs(s).SeparateFreeDofsIntoBoundaryAndInternal());
			dofManager.FindCommonDofsBetweenSubdomains();
			DistributedOverlappingIndexer indexer = dofManager.CreateDistributedVectorIndexer();

			// Check
			Line1DExample.CheckDistributedIndexer(environment, nodeTopology, indexer);
		}

		[Theory]
		[InlineData(EnvironmentChoice.SequentialSharedEnvironment)]
		[InlineData(EnvironmentChoice.TplSharedEnvironment)]
		public static void TestForPlane2D(EnvironmentChoice environmentChoice)
			=> TestForPlane2DInternal(Utilities.CreateEnvironment(environmentChoice));

		internal static void TestForPlane2DInternal(IComputeEnvironment environment)
		{
			ComputeNodeTopology nodeTopology = Plane2DExample.CreateNodeTopology();
			environment.Initialize(nodeTopology);

			IModel model = Plane2DExample.CreateMultiSubdomainModel();
			model.ConnectDataStructures();
			var subdomainTopology = new SubdomainTopology(environment, model);

			IGlobalFreeDofOrdering dofOrdering = ModelUtilities.OrderDofs(model);
			var dofManager = new PsmDofManager(environment, model, subdomainTopology,
				s => dofOrdering.SubdomainDofOrderings[s], true);
			environment.DoPerNode(s => dofManager.GetSubdomainDofs(s).SeparateFreeDofsIntoBoundaryAndInternal());
			dofManager.FindCommonDofsBetweenSubdomains();
			DistributedOverlappingIndexer indexer = dofManager.CreateDistributedVectorIndexer();

			// Check
			Plane2DExample.CheckDistributedIndexer(environment, nodeTopology, indexer);
		}
	}
}
