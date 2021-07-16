using System;
using MGroup.LinearAlgebra.Distributed;
using MGroup.LinearAlgebra.Vectors;

namespace MGroup.MSolve.AnalysisWorkflow.Logging
{
    public interface IAnalysisWorkflowLog
    {
        void StoreResults(DateTime startTime, DateTime endTime, IGlobalVector solution);
    }
}
