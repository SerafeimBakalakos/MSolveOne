using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Extensions
{
    /// <summary>
    /// Extension methods for <see cref="LinkedList{T}"/>.
    /// </summary>
    public static class LinkedListExtensions
    {
        /// <summary>
        /// Finds the first occurence of an entry that satisfies <paramref name="predicate"/>, removes it from the list and 
        /// returns it and true. Otherwise returns false.
        /// </summary>
        /// <typeparam name="T">Any type.</typeparam>
        /// <param name="list">The linked list.</param>
        /// <param name="predicate">The condition that the target entry must satisfy.</param>
        /// <param name="target">
        /// The first entry that satisfies <paramref name="predicate"/>. If no such entry exists, then this will be the 
        /// default value.
        /// </param>
        public static bool TryExtract<T>(this LinkedList<T> list, Predicate<T> predicate, out T target)
        {
            LinkedListNode<T> targetNode = null;
            LinkedListNode<T> currentNode = list.First;
            while (currentNode != null)
            {
                if (predicate(currentNode.Value))
                {
                    targetNode = currentNode;
                    break;
                }
                currentNode = currentNode.Next;
            }

            if (targetNode != null)
            {
                target = targetNode.Value;
                list.Remove(targetNode);
                return true;
            }
            else
            {
                target = default(T);
                return false;
            }
        }
    }
}
