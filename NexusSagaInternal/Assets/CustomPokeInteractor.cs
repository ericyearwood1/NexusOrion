using UnityEngine;
using Oculus.Interaction;

public class CustomPokeInteractor : PokeInteractor
{
    // Define a UnityEvent to notify when a poke occurs
    public UnityEngine.Events.UnityEvent OnPoke;

    protected override void Awake()
    {
        base.Awake();
        // Initialize the event if it's not already set
        if (OnPoke == null)
        {
            OnPoke = new UnityEngine.Events.UnityEvent();
        }
    }

    protected override void InteractableSelected(PokeInteractable interactable)
    {
        base.InteractableSelected(interactable);
        // Invoke the OnPoke event when an interactable is selected
        //OnPoke.Invoke();
        Poked(interactable);
    }

    protected override bool ComputeShouldSelect()
    {
        // Call the base method to determine if selection should occur
        bool shouldSelect = base.ComputeShouldSelect();

        // Additional logic can be added here if needed

        return shouldSelect;
    }

    public void Poked(PokeInteractable interactable)
    {
        PokeResponse pokeResponse = interactable.GetComponent<PokeResponse>();
        pokeResponse.OnPoke();
    }
}