using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Cracks.Geometry;
using MGroup.XFEM.Cracks.PropagationCriteria;

//TODO: This design is only good for checking data for each crack tip. Other stuff, like checking iterations must be done 
//      separately. Iterations are indeed an analyzer property so it may not be a big deal.
namespace MGroup.XFEM.Cracks.PropagationTermination
{
    /// <summary>
    /// These are push observers for ICrack classes.
    /// </summary>
    public interface IPropagationTermination
    {
        string Description { get; }

        bool MustTerminate(ICrack crack); 

        void Update(double[] sifs, double[] newCrackTip);
    }
}
