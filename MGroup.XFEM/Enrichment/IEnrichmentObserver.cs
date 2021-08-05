using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Enrichment
{
    public interface IEnrichmentObserver
    {
        void Update(IEnumerable<EnrichmentItem> allEnrichments);

        /// <summary>
        /// Observers will be called in the order they register. Therefore if an observer depends on the updated state of another
        /// the independent one must register first.
        /// </summary>
        /// <returns></returns>
        IEnrichmentObserver[] RegisterAfterThese();

    }
}
