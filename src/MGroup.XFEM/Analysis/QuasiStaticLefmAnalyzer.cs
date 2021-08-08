using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.Constitutive.Structural;
using MGroup.LinearAlgebra.Distributed;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Solution;
using MGroup.MSolve.Solution.AlgebraicModel;
using MGroup.Solvers.LinearSystem;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Cracks.PropagationTermination;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Analysis
{
	public class QuasiStaticLefmAnalyzer
	{
		private readonly XModel<IXCrackElement> model;
		private readonly IAlgebraicModel algebraicModel;
		private readonly ElementStructuralStiffnessProvider elementMatrixProvider;
		private readonly ISolver solver;
		private readonly int maxIterations;
		private readonly IPropagationTermination termination;

		public QuasiStaticLefmAnalyzer(XModel<IXCrackElement> model, IAlgebraicModel algebraicModel, ISolver solver, 
			int maxIterations, IPropagationTermination terminationCriterion)
		{
			this.model = model;
			this.algebraicModel = algebraicModel;
			this.solver = solver;
			this.maxIterations = maxIterations;
			this.termination = terminationCriterion;
			this.elementMatrixProvider = new ElementStructuralStiffnessProvider();
		}

		public string Status { get; private set; } = "Preparing";

		public void Analyze()
		{
			Status = "Running analysis";
			IGlobalVector totalDisplacementsFreeDofs = null;
			for (int iteration = 0; iteration < maxIterations; ++iteration)
			{
				Debug.WriteLine($"Crack propagation step {iteration}");

				if (iteration == 0) model.Initialize();
				else
				{
					model.Update(algebraicModel, totalDisplacementsFreeDofs);
					foreach (ICrack crack in model.GeometryModel.EnumerateDiscontinuities())
					{
						if (termination.MustTerminate(crack))
						{
							Status = 
								$"Terminated at iteration {iteration}, because of crack {crack.ID}: {termination.Description}";
							return;
						}

					}
				}

				algebraicModel.OrderDofs();

				// Create the stiffness matrix and forces vector
				BuildMatrices();
				AddLoadsToRhs();

				// Plot domain decomposition data, if necessary
				//if (DDLogger != null) DDLogger.PlotSubdomains(model);

				// Solve the linear system
				solver.Solve();
				totalDisplacementsFreeDofs = solver.LinearSystem.Solution;
			}
			Status = $"Termination after all {maxIterations} required iterations completed.";
		}

		private void AddLoadsToRhs()
		{
			// Nodal loads
			algebraicModel.AddToGlobalVector(model.EnumerateNodalLoads, solver.LinearSystem.RhsVector);

			// Dirichlet loads
			IGlobalVector dirichletLoads = algebraicModel.CreateZeroVector();
			algebraicModel.AddToGlobalVector(model.EnumerateDirichletBoundaryConditions, dirichletLoads);
			solver.LinearSystem.RhsVector.SubtractIntoThis(dirichletLoads);
		}

		private void BuildMatrices()
		{
			IGlobalMatrix Kff = algebraicModel.BuildGlobalMatrix(model.EnumerateElements, elementMatrixProvider);
			solver.LinearSystem.Matrix = Kff;
		}
	}
}
