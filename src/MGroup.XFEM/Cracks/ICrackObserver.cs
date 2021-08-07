using System;
using System.Collections.Generic;
using System.Text;

//TODO: Update() should not be called by the objervable object itself, in the middle of the model update/analysis. Instead, the 
//      model should store observables somewhere and call IObservable.NotifyObservers() at a specific point. In that design there
//      is no reason to differentiate between IEnrichmentObserver and ICrackObserver, especially if the observable is inserted
//      into the observer's constructor.
//      Advantages: 
//      1) The observable is not responsible for notifying the observers in 1 or more of its methods. 
//      2) Also many observers will write data to disc. In single threaded programs or when counting durations, it is better to 
//      not have these operations interfere with the model update or analysis. 
//      3) All observers are updated with the same model state, perhaps in a predefined order. Therefore there is no ambiguity in
//      when their data was extracted. Furthermore they can pull data from other observers, as long as they know that the other
//      observers are updated before themselves
//      Possible drawbacks: 
//      1) Perhaps delegating IO to other threads is actually more efficient during the analysis / model update. 
//      2) It is a more rigid design: E.g. it forces observers to update only after all model state has been updated for the 
//      current configuration. That may cause problems or inefficiencies for observers that track how specific state changes with 
//      respect to other state. At the time of writing, these limitations seem favourable, to avoid the chaos if any observer 
//      could be called at any time. Also there have always been workarounds for intermediate state so far, but they require 
//      extra computational work. 
//      ASK GOAT.
namespace MGroup.XFEM.Cracks
{
    public interface ICrackObserver
    {
        void Update();
    }
}
