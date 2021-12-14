using MGroup.XFEM.IsoXFEM.Solvers;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using System;
using System.Collections.Generic;
using System.Text;
using MGroup.MSolve.Discretization.Dofs;

namespace MGroup.XFEM.IsoXFEM
{
	public class FEMAnalysis
	{
		private readonly ISolver solver;
		private  Vector load;
		public Vector displacements;
		private GeometryProperties geometry;
		private readonly XModel<IsoXfemElement2D> xModel;
		private Dictionary<string, int[]> constraintsOfDofs = new Dictionary<string, int[]>();
		private int dofLoad;
		public /*private*/ Matrix globalStiffness;

        public FEMAnalysis(GeometryProperties geometry, XModel<IsoXfemElement2D> xModel, ISolver solver)
        {
			this.geometry = geometry;
			this.xModel = xModel;
            this.solver = solver;
        }
		private void CalcRHS()
		{
			 load = Vector.CreateZero(2 * (geometry.numberOfElementsX + 1) * (geometry.numberOfElementsY + 1));
			foreach (var nodalload in xModel.NodalLoads)
			{
			   if (nodalload.DOF== StructuralDof.TranslationX)
				{
					 dofLoad = 2 * nodalload.Node.ID; 
				}
			   else if (nodalload.DOF == StructuralDof.TranslationY)
				{
					 dofLoad = 2 * nodalload.Node.ID+1;
				}
				load[dofLoad] = nodalload.Amount;
			}
		}
		public /*private*/ void AssembleStiffnessMatrix()
        {
            globalStiffness = Matrix.CreateZero(2 * xModel.Nodes.Count, 2 * xModel.Nodes.Count);
            for (int i = 0; i < xModel.Elements.Count; i++)
            {
                var stiffnessElement = xModel.Elements[i].stiffnessOfElement;
                var dofs = xModel.Elements[i].dofsOfElement;
                for (int j = 0; j < dofs.Length; j++)
                {
                    for (int k = 0; k < dofs.Length; k++)
                    {
                        globalStiffness[dofs[j], dofs[k]] += stiffnessElement[j, k];
                    }
                }
            }
        }
		private void EnumerateDegreesOfFreedom()
		{
			var fixedDofs = new int[2 * (geometry.numberOfElementsY + 1)];
			var allDofs = new int[2 * xModel.Nodes.Count];
			for (int i = 0; i < 2 * (geometry.numberOfElementsY + 1); i++)
			{
				fixedDofs[i] = i;
			}
			for (int i = 0; i < 2 * xModel.Nodes.Count; i++)
			{
				allDofs[i] = i;
			}
			var freeDofs = ArraysMethods.SetDiff(fixedDofs, allDofs);
			constraintsOfDofs.Add("FreeDofs", freeDofs);
			constraintsOfDofs.Add("FixedDofs", fixedDofs);
			constraintsOfDofs.Add("AllDofs", allDofs);
		}
		public void RefillDisplacements(Vector solution)
        {
            int[] numOfFreedofs = new int[solution.Length];
            for (int i = 0; i < solution.Length; i++)
            {
                numOfFreedofs[i] = i;
            }
            displacements = Vector.CreateZero(constraintsOfDofs["AllDofs"].Length);
            displacements.AddIntoThisNonContiguouslyFrom(constraintsOfDofs["FreeDofs"], solution, numOfFreedofs);
        }

		public void Initialize()
		{
			CalcRHS();
			EnumerateDegreesOfFreedom();
		}
        public void Solve()
        {
			//EnumerateDegreesOfFreedom();
			AssembleStiffnessMatrix();
            Matrix globalStiffnessFree = globalStiffness.GetSubmatrix(constraintsOfDofs["FreeDofs"], constraintsOfDofs["FreeDofs"]);
            Vector loadFree = load.GetSubvector(constraintsOfDofs["FreeDofs"]);            
            Vector solution = solver.Solve(globalStiffnessFree, loadFree);
            RefillDisplacements(solution);
        }
    }
}
