namespace ARGlasses.Interaction
{
    public static class ExtensionsInputCategory
    {
        public static InputCategory Default => InputCategory.Eyes;

        public static bool IsNone(this InputCategory category) => category == InputCategory.None;
        public static bool IsEyes(this InputCategory category) => category == InputCategory.Eyes;
        public static bool IsHands(this InputCategory category) => category == InputCategory.Hands;
        public static bool IsCursor(this InputCategory category) => category == InputCategory.Cursor;
        public static bool IsCardinal(this InputCategory category) => category == InputCategory.Cardinal;

        public static string PrettyName(this InputCategory category)
        {
            // todo placeholder
            return category.ToString();
        }

        public static InputCategory Next(this InputCategory category)
        {
            if (category.IsEyes()) return InputCategory.Hands;
            if (category.IsHands()) return InputCategory.Cursor;
            if (category.IsCursor()) return InputCategory.Cardinal;
            if (category.IsCardinal()) return InputCategory.Eyes;
            return Default;
        }


        public static InputCategory Previous(this InputCategory category)
        {
            if (category.IsEyes()) return InputCategory.Cardinal;
            if (category.IsHands()) return InputCategory.Eyes;
            if (category.IsCursor()) return InputCategory.Hands;
            if (category.IsCardinal()) return InputCategory.Cursor;
            return Default;
        }
    }
    public enum InputCategory
    {
        None,
        Eyes,
        Hands,
        Cursor,
        Cardinal
    }
}
