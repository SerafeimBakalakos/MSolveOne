using System;
using System.Collections.Generic;
using System.Text;
using MGroup.LinearAlgebra.Matrices;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.LSM;
using MGroup.XFEM.Geometry.Primitives;
using MGroup.XFEM.Phases;

//TODO: I should test this independently in 1D and 2D elements, by mocking the elements. The result that should be checked is
//      the graph of psi(x), Ni(x)*psi(x) (shifted and not), gradient psix(x), gradient (Ni(x)*psi(x)),x
namespace MGroup.XFEM.Enrichment.Functions
{
    /// <summary>
    /// Enrichment scheme for material interfaces with weak discontinuities. Contrary to the ramp enrichment this scheme does not
    /// produce blending elements. See "A computational approach to handle complex microstructure geometries, Moes et al., 2003".
    /// </summary>
    /// <remarks>
    /// This enrichment is applied without shifting the nodal values, which are 0. However shifting them has no effect. 
    /// The computational cost is too negligible to justify altering the whole design of XFEM element and enrichment classes.
    /// </remarks>
    public class RidgeEnrichment : IEnrichmentFunction
    {
        private readonly IPhaseBoundary boundary;

        public RidgeEnrichment(IPhaseBoundary boundary)
        {
            this.boundary = boundary;
        }

        public EvaluatedFunction EvaluateAllAt(XPoint point)
        {
            // psi(x) = sum(|phi_i|*Ni(x)) - |sum(phi_i*Ni(x))|, 
            // psi,x(x) = sum(|phi_i|*Ni,x(x)) - sign( sum(phi_i*Ni(x)) ) * sum(phi_i*Ni,x(x))
            // where i belongs to element nodes, psi(x) = enrichment, phi_i = signed distance of node i
            int dim = point.Dimension;
            double sum1 = 0.0; // sum(|phi_i|*Ni(x))
            double sum2 = 0.0; // sum(phi_i*Ni(x))
            var sum1Grad = new double[dim]; // sum(|phi_i|*Ni,x(x))
            var sum2Grad = new double[dim]; // sum(phi_i*Ni,x(x))`
            Matrix gradN = point.ShapeFunctionDerivatives;
            for (int n = 0; n < point.Element.Nodes.Count; ++n)
            {
                XNode node = point.Element.Nodes[n];
                double phi = boundary.Geometry.SignedDistanceOf(node);
                double absPhi = Math.Abs(phi);
                double N = point.ShapeFunctions[n];

                sum1 += absPhi * N;
                sum2 += phi * N;

                for (int d = 0; d < dim; ++d)
                {
                    double derivativeN = gradN[n, d];
                    sum1Grad[d] += absPhi * derivativeN;
                    sum2Grad[d] += phi * derivativeN;
                }
            }

            double value = sum1 - Math.Abs(sum2);
            var gradient = new double[dim];
            for (int d = 0; d < dim; ++d)
            {
                gradient[d] = sum1Grad[d] - Math.Sign(sum2) * sum2Grad[d];
            }
            return new EvaluatedFunction(value, gradient);
        }

        public double EvaluateAt(XNode node)
        {
            // Due to its construction, the enrichment becomes zero at nodes
            return 0.0;
        }

        public double EvaluateAt(XPoint point)
        {
            // psi(x) = sum(|phi_i|*Ni(x)) - |sum(phi_i*Ni(x))|, 
            // where i belongs to element nodes, psi(x) = enrichment, phi(x) = signed distance of node i
            double sum1 = 0.0; // sum(|phi_i|*Ni(x))
            double sum2 = 0.0; // sum(phi_i*Ni(x))
            for (int n = 0; n < point.Element.Nodes.Count; ++n)
            {
                XNode node = point.Element.Nodes[n];
                double phi = boundary.Geometry.SignedDistanceOf(node);
                double N = point.ShapeFunctions[n];
                sum1 += Math.Abs(phi) * N;
                sum2 += phi * N;
            }
            return sum1 - Math.Abs(sum2);
        }

        public double EvaluateJumpAcross(IXDiscontinuity discontinuity, XPoint point)
        {
            // There is no jump. Do not use this enrichment for problems with discontinuous primary fields. 
            return 0;
        }
    }
}
