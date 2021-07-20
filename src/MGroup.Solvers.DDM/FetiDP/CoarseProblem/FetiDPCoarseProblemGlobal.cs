using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Text;
using MGroup.Environments;
using MGroup.LinearAlgebra.Distributed.Overlapping;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.Solvers.DDM.Commons;
using MGroup.Solvers.DDM.FetiDP.Dofs;
using MGroup.Solvers.DDM.FetiDP.StiffnessMatrices;

namespace MGroup.Solvers.DDM.FetiDP.CoarseProblem
{
	public class FetiDPCoarseProblem : IFetiDPCoarseProblem
	{
		private readonly IComputeEnvironment environment;
		private readonly IModel model;
		private readonly IFetiDPCoarseProblemMatrix coarseProblemMatrix;
		private readonly Func<int, FetiDPSubdomainDofs> getSubdomainDofs;
		private readonly Func<int, IFetiDPSubdomainMatrixManager> getSubdomainMatrices;
		private readonly FetiDPCoarseProblemDofsGlobal globalDofs;

		public FetiDPCoarseProblem(IComputeEnvironment environment, IModel model, IFetiDPCoarseProblemMatrix coarseProblemMatrix,
			Func<int, FetiDPSubdomainDofs> getSubdomainDofs, Func<int, IFetiDPSubdomainMatrixManager> getSubdomainMatrices)
		{
			this.environment = environment;
			this.model = model;
			this.coarseProblemMatrix = coarseProblemMatrix;
			this.getSubdomainDofs = getSubdomainDofs;
			this.getSubdomainMatrices = getSubdomainMatrices;
			this.globalDofs = new FetiDPCoarseProblemDofsGlobal(environment, model, getSubdomainDofs);
		}

		public void FindCoarseProblemDofs()
		{
			globalDofs.FindGlobalCornerDofs();
			globalDofs.CalcSubdomainGlobalCornerDofMaps();
			DofPermutation permutation = coarseProblemMatrix.ReorderGlobalCornerDofs(
				globalDofs.NumGlobalCornerDofs, globalDofs.SubdomainToGlobalCornerDofMaps);
			globalDofs.ReorderGlobalCornerDofs(permutation);
		}

		public void PrepareMatricesForSolution()
		{
			Dictionary<int, IMatrix> subdomainMatricesScc = GatherSubdomainMatricesScc();
			coarseProblemMatrix.InvertGlobalScc(
				globalDofs.NumGlobalCornerDofs, globalDofs.SubdomainToGlobalCornerDofMaps, subdomainMatricesScc);
		}

		public void SolveCoarseProblem(IDictionary<int, Vector> coarseProblemRhs, IDictionary<int, Vector> coarseProblemSolution)
		{
			// Map reduce subdomain vectors to global
			Dictionary<int, Vector> subdomainRhsVectors = GatherSubdomainVectors(coarseProblemRhs);
			var globalRhs = Vector.CreateZero(globalDofs.NumGlobalCornerDofs); 
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				int s = subdomain.ID;
				Vector subdomainRhs = subdomainRhsVectors[s];

				int[] subdomainToGlobalMap = globalDofs.SubdomainToGlobalCornerDofMaps[s];
				globalRhs.AddIntoThisNonContiguouslyFrom(subdomainToGlobalMap, subdomainRhs);

				//TODOMPI: delete this and the Lc matrices. They are used below, but this is not faser than the code above.
				//Mappings.IMappingMatrix Lc = globalDofs.SubdomainMatricesLc[s];
				//globalRhs.AddIntoThis(Lc.Multiply(subdomainRhs, true));
			}

			// Solve global problem
			var globalSolution = Vector.CreateZero(globalDofs.NumGlobalCornerDofs);
			coarseProblemMatrix.MultiplyInverseScc(globalRhs, globalSolution);

			// Distribute global solution to subdomains
			var subdomainSolutionVectors = new Dictionary<int, Vector>();
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				int s = subdomain.ID;

				int[] subdomainToGlobalMap = globalDofs.SubdomainToGlobalCornerDofMaps[s];
				Vector subdomainSolution = globalSolution.GetSubvector(subdomainToGlobalMap);

				//TODO: delete this and the Lc matrices. They are used below, but this is not faster than the code above.
				//Mappings.IMappingMatrix Lc = globalDofs.SubdomainMatricesLc[s];
				//Vector subdomainSolution = Lc.Multiply(globalSolution, false);

				subdomainSolutionVectors[s] = subdomainSolution;
			}
			ScatterSubdomainVectors(subdomainSolutionVectors, coarseProblemSolution);
		}

		#region TODOMPI: these should be done by the environment or by a dedicated class that bridges the environment and coarse problems/global operations
		private Dictionary<int, IMatrix> GatherSubdomainMatricesScc()
		{
			var subdomainMatricesScc = new Dictionary<int, IMatrix>();
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				subdomainMatricesScc[subdomain.ID] = getSubdomainMatrices(subdomain.ID).SchurComplementOfRemainderDofs;
			}
			return subdomainMatricesScc;
		}

		private Dictionary<int, Vector> GatherSubdomainVectors(IDictionary<int, Vector> coarseProblemRhs)
		{
			var localVectors = new Dictionary<int, Vector>();
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				int s = subdomain.ID;
				localVectors[s] = coarseProblemRhs[s];
			}
			return localVectors;
		}

		private void ScatterSubdomainVectors(Dictionary<int, Vector> localVectors, IDictionary<int, Vector> remoteVectors)
		{
			if (remoteVectors.Count > 0)
			{
				throw new NotImplementedException();
			}
			foreach (ISubdomain subdomain in model.EnumerateSubdomains())
			{
				int s = subdomain.ID;
				remoteVectors[s] = localVectors[s];
			}
		}
		#endregion
	}
}
