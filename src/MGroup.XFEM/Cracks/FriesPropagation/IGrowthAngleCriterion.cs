using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Cracks.FriesPropagation
{
	public interface IGrowthAngleCriterion
	{
		void FindPropagationAngle(TrialPointStresses trialPointStresses);
	}
}
