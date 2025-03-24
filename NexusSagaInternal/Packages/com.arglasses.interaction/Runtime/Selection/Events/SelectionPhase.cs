// (c) Meta Platforms, Inc. and affiliates. Confidential and proprietary.

using static ARGlasses.Interaction.SelectionPhase;

namespace ARGlasses.Interaction
{
    public static class SelectionPhaseExtensions
    {
        public static bool IsNone(this SelectionPhase phase) => phase == None;
        public static bool IsBegin(this SelectionPhase phase) => phase == Begin;
        public static bool IsUpdate(this SelectionPhase phase) => phase == Update;
        public static bool IsSuccess(this SelectionPhase phase) => phase == Success;
        public static bool IsCancel(this SelectionPhase phase) => phase == Cancel;
        public static bool IsEnded(this SelectionPhase phase) => phase.IsSuccess() || phase.IsCancel();
    }

    public enum SelectionPhase
    {
        None,
        Begin, // same frame as press, Current and Begin will be the same
        Update,
        Success, // same frame as release
        Cancel,
    }
}
