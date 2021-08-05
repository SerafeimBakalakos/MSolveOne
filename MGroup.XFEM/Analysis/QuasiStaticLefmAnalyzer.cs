using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.Constitutive.Structural;
using MGroup.LinearAlgebra.Matrices;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Solution;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Cracks.PropagationTermination;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Analysis
{
    public class QuasiStaticLefmAnalyzer
    {
        private readonly XModel<IXCrackElement> model;
        private readonly ISolver solver;
        private readonly int maxIterations;
        private readonly IPropagationTermination termination;
        private readonly ElementStructuralStiffnessProvider problem;
        private readonly DirichletEquivalentLoadsStructural loadsAssembler;

        public QuasiStaticLefmAnalyzer(XModel<IXCrackElement> model, ISolver solver, int maxIterations, 
            IPropagationTermination terminationCriterion)
        {
            this.model = model;
            this.solver = solver;
            this.maxIterations = maxIterations;
            this.termination = terminationCriterion;
            this.problem = new ElementStructuralStiffnessProvider();
            this.loadsAssembler = new DirichletEquivalentLoadsStructural(problem);
        }

        public string Status { get; private set; } = "Preparing";

        public void Analyze()
        {
            Status = "Running analysis";
            Dictionary<int, Vector> freeDisplacements = null;
            for (int iteration = 0; iteration < maxIterations; ++iteration)
            {
                Debug.WriteLine($"Crack propagation step {iteration}");

                if (iteration == 0) model.Initialize();
                else
                {
                    model.Update(freeDisplacements);
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

                solver.OrderDofs(false);
                foreach (ILinearSystem linearSystem in solver.LinearSystems.Values)
                {
                    //if (linearSystem.Subdomain.ConnectivityModified)
                    //{
                        linearSystem.Reset(); // Necessary to define the linear system's size 
                        linearSystem.Subdomain.Forces = Vector.CreateZero(linearSystem.Size);
                    //}
                }

                // Create the stiffness matrix and then the forces vector
                //problem.ClearMatrices();
                BuildMatrices();
                model.AssignLoads(solver.DistributeNodalLoads);
                foreach (ILinearSystem linearSystem in solver.LinearSystems.Values)
                {
                    linearSystem.RhsVector = linearSystem.Subdomain.Forces;
                }
                AddEquivalentNodalLoadsToRhs();

                // Plot domain decomposition data, if necessary
                //if (DDLogger != null) DDLogger.PlotSubdomains(model);

                // Solve the linear system
                solver.Solve();
                freeDisplacements = new Dictionary<int, Vector>();
                foreach (int s in solver.LinearSystems.Keys)
                {
                    freeDisplacements[s] = (Vector)(solver.LinearSystems[s].Solution); //TODO: avoid this cast.
                }
            }
            Status = $"Termination after all {maxIterations} required iterations completed.";
        }

        private void AddEquivalentNodalLoadsToRhs()
        {
            foreach (ILinearSystem linearSystem in solver.LinearSystems.Values)
            {
                try
                {
                    // Make sure there is at least one non zero prescribed displacement.
                    (INode node, IDofType dof, double displacement) = linearSystem.Subdomain.Constraints.Find(du => du != 0.0);

                    //TODO: the following 2 lines are meaningless outside diplacement control (and even then, they are not so clear).
                    double scalingFactor = 1;
                    IVector initialFreeSolution = linearSystem.CreateZeroVector();

                    //IVector equivalentNodalLoads = problem.DirichletLoadsAssembler.GetEquivalentNodalLoads(
                    //    linearSystem.Subdomain, initialFreeSolution, scalingFactor);
                    IVector equivalentNodalLoads = loadsAssembler.GetEquivalentNodalLoads(linearSystem.Subdomain,
                        initialFreeSolution, scalingFactor);
                    linearSystem.RhsVector.SubtractIntoThis(equivalentNodalLoads);
                }
                catch (KeyNotFoundException)
                {
                    // There aren't any non zero prescribed displacements, therefore we do not have to calculate the equivalent 
                    // nodal loads, which is an expensive operation (all elements are accessed, their stiffness is built, etc..)
                }
            }
        }

        private void BuildMatrices()
        {
            Dictionary<int, IMatrix> stiffnesses = solver.BuildGlobalMatrices(problem);
            foreach (ILinearSystem linearSystem in solver.LinearSystems.Values)
            {
                linearSystem.Matrix = stiffnesses[linearSystem.Subdomain.ID];
            }
        }

    }
}
