using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using OSIG.Tools.Layout;
using OSIG.Tools.Layout.Internals;

public class OCSliderAttachData : OCInsetAttachData, IAttachData<OCSliderAttachData.AttachStruct> {

    public FillType Type;

    public void GetAttachStruct(out AttachStruct t) {
        t.Type = Type;
        GetAttachStruct(out t.Inset);
    }

    public struct AttachStruct : IAttachStruct, IAttachData<OCLayoutInset.AttachStruct> {

        public FillType Type;
        public OCLayoutInset.AttachStruct Inset;

        public void GetAttachStruct(out OCLayoutInset.AttachStruct t) {
            t = Inset;
        }
    }

    public enum FillType {
        InclusiveFill,
        ExclusiveFill,
        Handle
    }
}
