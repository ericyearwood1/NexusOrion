namespace FigmaUnity.FigmaComponents
{
    //Ensure implementers are value types, such as structs
    //TODO: Figure out why this can't be a struct
    public interface IComponentStyle 
    {
        void FromString(string componentStyleString);
    }
}