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
using MGroup.XFEM.Output.Writers;

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
		private readonly bool reanalysis;

		public QuasiStaticLefmAnalyzer(XModel<IXCrackElement> model, IAlgebraicModel algebraicModel, ISolver solver, 
			int maxIterations, IPropagationTermination terminationCriterion, bool reanalysis = true)
		{
			this.model = model;
			this.algebraicModel = algebraicModel;
			this.solver = solver;
			this.maxIterations = maxIterations;
			this.termination = terminationCriterion;
			this.reanalysis = reanalysis;
			this.elementMatrixProvider = new ElementStructuralStiffnessProvider();
		}

		public List<IResultsWriter> Results { get; } = new List<IResultsWriter>();

		public string Status { get; private set; } = "Preparing";

		public void Analyze()
		{
			Status = "Running analysis";
			IGlobalVector totalDisplacementsFreeDofs = null;
			for (int iteration = 0; iteration < maxIterations; ++iteration)
			{
				Debug.WriteLine($"Crack propagation step {iteration}");
				Console.WriteLine($"Crack propagation step {iteration}");

				if (iteration == 0)
				{
					model.Initialize();
					algebraicModel.OrderDofs();
					BuildMatrices();
				}
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

					if (reanalysis)
					{
						algebraicModel.ReorderDofs();
						RebuildMatrices();
					}
					else
					{
						algebraicModel.OrderDofs();
						BuildMatrices();
					}
				}

				AddLoadsToRhs();

				// Plot domain decomposition data, if necessary
				//if (DDLogger != null) DDLogger.PlotSubdomains(model);

				// Solve the linear system
				solver.Solve();
				totalDisplacementsFreeDofs = solver.LinearSystem.Solution;
				foreach (IResultsWriter writer in Results)
				{
					writer.WriteResults(algebraicModel, totalDisplacementsFreeDofs);
				}
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

		private void RebuildMatrices()
		{
			IGlobalMatrix previousKff = solver.LinearSystem.Matrix;
			solver.LinearSystem.Matrix = algebraicModel.RebuildGlobalMatrixPartially(
				previousKff, model.EnumerateElements, elementMatrixProvider);
		}
	}
}
