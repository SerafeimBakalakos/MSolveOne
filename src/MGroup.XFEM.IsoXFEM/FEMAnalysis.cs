using MGroup.XFEM.IsoXFEM.Solvers;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.IsoXFEM
{
    public class FEMAnalysis
    {
        private readonly ISolver solver;
        private readonly Vector load;
        public Vector displacements;
        private readonly Model model;
		public /*private*/ Matrix globalStiffness;

        public FEMAnalysis(Model model, ISolver solver,Vector load)
        {
            this.model = model;
            this.solver = solver;
            this.load = load;
        }
		public /*private*/ void AssembleStiffnessMatrix()
        {
            globalStiffness = Matrix.CreateZero(2 * model.nodes.Count, 2 * model.nodes.Count);
            for (int i = 0; i < model.elements.Count; i++)
            {
                var stiffnessElement = model.elements[i].stiffnessOfElement;
                var dofs = model.elements[i].dofsOfElement;
                for (int j = 0; j < dofs.Length; j++)
                {
                    for (int k = 0; k < dofs.Length; k++)
                    {
                        globalStiffness[dofs[j], dofs[k]] += stiffnessElement[j, k];
                    }
                }
            }
        }
        public void RefillDisplacements(Vector solution)
        {
            int[] numOfFreedofs = new int[solution.Length];
            for (int i = 0; i < solution.Length; i++)
            {
                numOfFreedofs[i] = i;
            }
            displacements = Vector.CreateZero(model.constraintsOfDofs["AllDofs"].Length);
            displacements.AddIntoThisNonContiguouslyFrom(model.constraintsOfDofs["FreeDofs"], solution, numOfFreedofs);
        }
        public void Solve()
        {
            AssembleStiffnessMatrix();
            Matrix globalStiffnessFree = globalStiffness.GetSubmatrix(model.constraintsOfDofs["FreeDofs"], model.constraintsOfDofs["FreeDofs"]);
            Vector loadFree = load.GetSubvector(model.constraintsOfDofs["FreeDofs"]);            
            Vector solution = solver.Solve(globalStiffnessFree, loadFree);
            RefillDisplacements(solution);
        }
    }
}
