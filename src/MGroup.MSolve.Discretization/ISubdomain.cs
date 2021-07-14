using System.Collections.Generic;
using MGroup.MSolve.DataStructures;

//TODO: tidy up the methods that concern material state
namespace MGroup.MSolve.Discretization
{
    public interface ISubdomain
    {
		IEnumerable<IElement> Elements { get; }

        int ID { get; }

        bool LinearSystemModified { get; set; }

        IReadOnlyList<INode> Nodes { get; } 

        void ResetMaterialsModifiedProperty();

        void SaveMaterialState();
    }
}
