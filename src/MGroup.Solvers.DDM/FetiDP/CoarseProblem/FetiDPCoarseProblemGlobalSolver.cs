using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Vectors;

namespace MGroup.Solvers.DDM.FetiDP.CoarseProblem
{
	public class FetiDPCoarseProblemGlobalSolver
	{
		private readonly IFetiDPCoarseProblemGlobalMatrix coarseProblemMatrix;
		private readonly FetiDPCoarseProblemGlobalDofs coarseProblemDofs;

		public FetiDPCoarseProblemGlobalSolver(FetiDPCoarseProblemGlobalDofs coarseProblemDofs,
			IFetiDPCoarseProblemGlobalMatrix coarseProblemMatrix)
		{
			this.coarseProblemDofs = coarseProblemDofs;
			this.coarseProblemMatrix = coarseProblemMatrix;
		}

		public void SolveCoarseProblem(IDictionary<int, Vector> coarseProblemRhs, IDictionary<int, Vector> coarseProblemSolution)
		{
			// Map reduce subdomain vectors to global
			var globalRhs = Vector.CreateZero(coarseProblemDofs.NumGlobalCornerDofs); 
			foreach (int s in coarseProblemRhs.Keys)
			{
				Vector subdomainRhs = coarseProblemRhs[s];

				int[] subdomainToGlobalDofs = coarseProblemDofs.SubdomainToGlobalCornerDofs[s];
				globalRhs.AddIntoThisNonContiguouslyFrom(subdomainToGlobalDofs, subdomainRhs);

				//TODOMPI: delete this and the Lc matrices. They are used below, but this is not faster than the code above.
				//Mappings.IMappingMatrix Lc = coarseProblemDofs.SubdomainMatricesLc[s];
				//globalRhs.AddIntoThis(Lc.Multiply(subdomainRhs, true));
			}

			// Solve global problem
			var globalSolution = Vector.CreateZero(coarseProblemDofs.NumGlobalCornerDofs);
			coarseProblemMatrix.MultiplyInverseScc(globalRhs, globalSolution);

			// Distribute global solution to subdomains
			foreach (int s in coarseProblemRhs.Keys)
			{
				int[] subdomainToGlobalDofs = coarseProblemDofs.SubdomainToGlobalCornerDofs[s];
				Vector subdomainSolution = globalSolution.GetSubvector(subdomainToGlobalDofs);

				//TODO: delete this and the Lc matrices. They are used below, but this is not faster than the code above.
				//Mappings.IMappingMatrix Lc = globalDofs.SubdomainMatricesLc[s];
				//Vector subdomainSolution = Lc.Multiply(globalSolution, false);

				coarseProblemSolution[s] = subdomainSolution; //TODO: write into the existing vector directly during GetSubvector()
			}
		}
	}
}
