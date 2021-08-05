using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Elements;
using MGroup.XFEM.Integration.Quadratures;

namespace MGroup.XFEM.Integration
{
    public class JintegrationStrategy : IBulkIntegration
    {
        private readonly IQuadrature stdElementIntegration;
        private readonly IBulkIntegration intersectedElementIntegration;

        public JintegrationStrategy(IQuadrature stdElementIntegration, IBulkIntegration intersectedElementIntegration)
        {
            this.stdElementIntegration = stdElementIntegration;
            this.intersectedElementIntegration = intersectedElementIntegration;
        }

        public IReadOnlyList<GaussPoint> GenerateIntegrationPoints(IXFiniteElement element)
        {
            if (!(element is IXCrackElement))
            {
                throw new ArgumentException();
            }

            var crackElement = (IXCrackElement)element;
            if (crackElement.IsIntersected())
            {
                return intersectedElementIntegration.GenerateIntegrationPoints(element);
            }
            else
            {
                return stdElementIntegration.IntegrationPoints;
            }
        }
    }
}
