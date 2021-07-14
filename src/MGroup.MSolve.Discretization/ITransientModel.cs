using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.MSolve.Discretization
{
	public interface ITransientModel : IModel
	{
		int TimeStep { get; set; } //TODO: Should the Analyzer set this or the Problem?
	}
}
