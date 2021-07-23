using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.Environments;
using MGroup.Environments.Mpi;
using MGroup.LinearAlgebra.Distributed.Overlapping;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.DDM.LinearSystem;
using MGroup.Solvers.DDM.PSM.Dofs;
using MGroup.Solvers.DDM.PSM.InterfaceProblem;
using MGroup.Solvers.DDM.Tests.ExampleModels;
using MGroup.Solvers.DofOrdering;
using Xunit;

namespace MGroup.Solvers.DDM.Tests.PSM
{
	public class PsmInterfaceProblemDofsTests
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
			DistributedOverlappingIndexer indexer = CreateDistributedOverlappingIndexer(environment, model);

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
			DistributedOverlappingIndexer indexer = CreateDistributedOverlappingIndexer(environment, model);

			// Check
			Plane2DExample.CheckDistributedIndexer(environment, nodeTopology, indexer);
		}

		private static DistributedOverlappingIndexer CreateDistributedOverlappingIndexer(
			IComputeEnvironment environment, IModel model)
		{
			model.ConnectDataStructures();

			Dictionary<int, ISubdomainFreeDofOrdering> dofOrderings = environment.CreateDictionaryPerNode(
				s => ModelUtilities.OrderDofs(model.GetSubdomain(s)));
			var subdomainTopology = new SubdomainTopology(environment, model, s => dofOrderings[s]);

			Dictionary<int, MockSubdomainLinearSystem> linearSystems = environment.CreateDictionaryPerNode(
				s => new MockSubdomainLinearSystem(dofOrderings[s]));
			Dictionary<int, PsmSubdomainDofs> subdomainDofs = environment.CreateDictionaryPerNode(
				s => new PsmSubdomainDofs(model.AllDofs, linearSystems[s], true));

			subdomainTopology.FindCommonNodesBetweenSubdomains();
			subdomainTopology.FindCommonDofsBetweenSubdomains();
			environment.DoPerNode(s => subdomainDofs[s].SeparateFreeDofsIntoBoundaryAndInternal());
			return subdomainTopology.CreateDistributedVectorIndexer(s => subdomainDofs[s].DofOrderingBoundary);
		}

		private class MockSubdomainLinearSystem : ISubdomainLinearSystem
		{
			public MockSubdomainLinearSystem(ISubdomainFreeDofOrdering dofOrdering)
			{
				this.DofOrdering = dofOrdering;
			}

			public ISubdomainFreeDofOrdering DofOrdering { get; }

			public IMatrix Matrix => throw new NotImplementedException();

			public Vector RhsVector => throw new NotImplementedException();

			public Vector Solution { get => throw new NotImplementedException(); set => throw new NotImplementedException(); }
		}
	}
}