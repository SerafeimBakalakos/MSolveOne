using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Cracks.Geometry;

namespace MGroup.XFEM.Cracks.PropagationTermination
{
    public static class TerminationLogic
    {
        public class And : IPropagationTermination
        {
            private readonly IPropagationTermination[] criteria;

            public And(params IPropagationTermination[] criteria)
            {
                this.criteria = criteria;
                var builder = new StringBuilder();
                builder.Append(criteria[0].Description);
                for (int i = 1; i < criteria.Length; ++i)
                {
                    builder.Append(" AND ");
                    builder.Append(criteria[i].Description);
                }
                Description = builder.ToString();
            }

            public string Description { get; }

            public bool MustTerminate(ICrack crack)
            {
                foreach (IPropagationTermination criterion in criteria)
                {
                    if (!criterion.MustTerminate(crack))
                    {
                        return false;
                    }
                }
                return true;
            }

            public void Update(double[] sifs, double[] newCrackTip)
            {
            }
        }

        public class Or : IPropagationTermination
        {
            private readonly IPropagationTermination[] criteria;

            public Or(params IPropagationTermination[] criteria)
            {
                this.criteria = criteria;
                var builder = new StringBuilder();
                builder.Append(criteria[0].Description);
                for (int i = 1; i < criteria.Length; ++i)
                {
                    builder.Append(" OR ");
                    builder.Append(criteria[i].Description);
                }
                Description = builder.ToString();
            }

            public string Description { get; }

            public bool MustTerminate(ICrack crack)
            {
                foreach (IPropagationTermination criterion in criteria)
                {
                    if (criterion.MustTerminate(crack))
                    {
                        return true;
                    }
                }
                return false;
            }

            public void Update(double[] sifs, double[] newCrackTip)
            {
            }
        }

        public class Not : IPropagationTermination
        {
            private readonly IPropagationTermination criterion;

            public Not(IPropagationTermination criterion)
            {
                this.criterion = criterion;
                Description = "NOT " + criterion.Description;
            }

            public string Description { get; }

            public bool MustTerminate(ICrack crack)
            {
                return !criterion.MustTerminate(crack);
            }

            public void Update(double[] sifs, double[] newCrackTip)
            {
            }
        }

        public class Null : IPropagationTermination
        {
            public string Description { get; } = "Do not terminate";

            public bool MustTerminate(ICrack crack) => false;

            public void Update(double[] sifs, double[] newCrackTip)
            {
            }
        }
    }
}
