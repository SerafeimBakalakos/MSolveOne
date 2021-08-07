using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MGroup.XFEM.Elements;

namespace MGroup.XFEM.Integration
{
    public class CrackElementIntegrationStrategy : IBulkIntegration
    {
        private readonly IBulkIntegration integrationForIntersectedHeavisideElements;
        private readonly IBulkIntegration integrationForTipElements;

        // TODO: verify the need to use higher order integration for tip blending elements
        private readonly IBulkIntegration integrationForTipBlendingElements;

        public CrackElementIntegrationStrategy(
            IBulkIntegration integrationForIntersectedHeavisideElements,
            IBulkIntegration integrationForTipElements,
            IBulkIntegration integrationForTipBlendingElements)
        {
            this.integrationForIntersectedHeavisideElements = integrationForIntersectedHeavisideElements;
            this.integrationForTipElements = integrationForTipElements;
            this.integrationForTipBlendingElements = integrationForTipBlendingElements;
        }

        public IReadOnlyList<GaussPoint> GenerateIntegrationPoints(IXFiniteElement element)
        {
            if (!(element is IXCrackElement))
            {
                throw new ArgumentException();
            }

            var crackElement = (IXCrackElement)element;
            if (!crackElement.HasEnrichedNodes())
            {
                // Case 0: Element without any enriched nodes. The standard FEM integration rule can be used, since all 
                // integrands are the FEM polynomials
                return crackElement.IntegrationStandard.IntegrationPoints;
            }
            else
            {
                bool isIntersectedElement = crackElement.IsIntersected();
                bool isTipElement = crackElement.IsTipElement();
                if (isTipElement)
                {
                    // Case 1: Element that is fully enriched with crack tip functions and may contain the crack tip itself.
                    // Special integration rules must be used that take into account: a) the crack tip enrichment functions,
                    // b) the fact that the element is intersected by a crack and thus integrands are discontinuous functions 
                    return integrationForTipElements.GenerateIntegrationPoints(crackElement);
                }
                else if (crackElement.HasTipEnrichedNodes())
                {
                    if (isIntersectedElement)
                    {
                        // Case 2: Blending element, with some nodes enriched with crack tip functions, while also being 
                        // intersected by the crack. The requirements are similar to Case 1.
                        return integrationForTipElements.GenerateIntegrationPoints(crackElement);
                    }
                    else
                    {
                        // Case 3: Blending element, with some nodes enriched with crack tip functions, without being 
                        // intersected by the crack. Contrary to cases 1 and 2, now we only have to take into account the crack
                        // tip enrichment functions.
                        return integrationForTipBlendingElements.GenerateIntegrationPoints(crackElement);
                    }
                }
                else if (isIntersectedElement)
                {
                    // Case 4: Element intersected by the crack and fully enriched with Heaviside functions. We need to take into
                    // acccount the discontinuous integrand functions.
                    return integrationForIntersectedHeavisideElements.GenerateIntegrationPoints(crackElement);
                }
                else
                {
                    // Case 5: Blending element, with some nodes being enriched with Heaviside functions. Since it is not 
                    // intersected by any cracks, all integrands are polynomials and the standard (FEM) integration rule can be 
                    // used
                    return crackElement.IntegrationStandard.IntegrationPoints;
                }
            }
        }
    }
}
