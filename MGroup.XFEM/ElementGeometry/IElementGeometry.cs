using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.ElementGeometry
{
    public interface IElementGeometry
    {
        /// <summary>
        /// 1D: calculates length. 2D: calculates area. 3D: calculates volume
        /// </summary>
        /// <param name="nodes"></param>
        double CalcBulkSizeCartesian(IReadOnlyList<XNode> nodes);

        /// <summary>
        /// 1D: calculates length. 2D: calculates area. 3D: calculates volume
        /// </summary>
        /// <param name="nodes"></param>
        double CalcBulkSizeNatural();

        (ElementEdge[], ElementFace[]) FindEdgesFaces(IReadOnlyList<int> nodeIDs);
    }
}
