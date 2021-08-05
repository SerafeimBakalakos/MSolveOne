using System.Collections.Generic;
using MGroup.XFEM.Integration;
using MGroup.XFEM.Elements;

//TODO: Tidy up integration rules for crack and multiphase problems. Some of them need to check the element state, while others
//      depend on the element to make decisions. Also a common interface should be used for all integrations. Alternatively,
//      IXFiniteElement should not expose its IBulkIntegration. DO NOT USE GENERICS for this interface.
namespace MGroup.XFEM.Integration
{
    /// <summary>
    /// Algorithms for complex integration rules for specific finite element types. These need the data from each 
    /// finite element to generate integration points for use only by that finite element. 
    /// They typically make use of the standard quadrature rules.
    /// </summary>
    public interface IBulkIntegration
    {
        IReadOnlyList<GaussPoint> GenerateIntegrationPoints(IXFiniteElement element);
    }
}
