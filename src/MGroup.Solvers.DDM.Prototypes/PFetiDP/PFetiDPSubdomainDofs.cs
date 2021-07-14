using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using MGroup.MSolve.Discretization;
using MGroup.LinearAlgebra.Matrices;
using MGroup.Solvers.DDM.Prototypes.FetiDP;
using MGroup.Solvers.DDM.Prototypes.LinearAlgebraExtensions;
using MGroup.Solvers.DDM.Prototypes.PSM;

namespace MGroup.Solvers.DDM.Prototypes.PFetiDP
{
    public class PFetiDPSubdomainDofs
    {
        protected readonly IModel model;
        protected readonly PsmSubdomainDofs psmDofs;
        protected readonly FetiDPSubdomainDofs fetiDPDofs;

        public PFetiDPSubdomainDofs(IModel model, PsmSubdomainDofs psmDofs, FetiDPSubdomainDofs fetiDPDofs)
        {
            this.model = model;
            this.psmDofs = psmDofs;
            this.fetiDPDofs = fetiDPDofs;
        }

        public BlockMatrix MatrixNrbe { get; set; }

        public Dictionary<int, Matrix> SubdomainMatricesNrb { get; } = new Dictionary<int, Matrix>();

        public void MapPsmFetiDPDofs()
        {
            MatrixNrbe = BlockMatrix.Create(fetiDPDofs.NumSubdomainDofsRemainder, psmDofs.NumSubdomainDofsBoundary);
            foreach (ISubdomain subdomain in model.EnumerateSubdomains())
            {
                int s = subdomain.ID;

                // Free to boundary dofs
                int[] boundaryToFree = psmDofs.SubdomainDofsBoundaryToFree[s];
                var freeToBoundary = new Dictionary<int, int>();
                for (int i = 0; i < boundaryToFree.Length; i++)
                {
                    freeToBoundary[boundaryToFree[i]] = i;
                }

                // Remainder to free dofs
                int[] remainderToFree = fetiDPDofs.SubdomainDofsRemainderToFree[s];
                var Nrb = Matrix.CreateZero(remainderToFree.Length, boundaryToFree.Length);
                for (int r = 0; r < remainderToFree.Length; r++)
                {
                    int f = remainderToFree[r];
                    bool exists = freeToBoundary.TryGetValue(f, out int b);
                    if (exists) // some remainder dofs are internal, thus they cannot be boundary too.
                    {
                        Nrb[r, b] = 1.0;
                    }
                }
                this.SubdomainMatricesNrb[s] = Nrb;
                this.MatrixNrbe.AddBlock(s, s, Nrb);
            }
        }
    }
}
