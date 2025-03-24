using Data;
using Prime31.StateKit;
using View;

namespace States
{
    public class ConfirmUserDetailsState : SKState<AppData>
    {
        private ConfirmExistingUserDetailsView _view;

        public override void begin()
        {
            base.begin();
            _view = _context.FullFocusCanvasUI.ShowConfirmExistingUserDetails();
            _view.Initialise(_context.MultiplayerData.ThisUser, _context.ColorConfig);
            _view.OnEdit += OnEditClicked;
            _view.OnConfirm += OnConfirmClicked;
        }

        private void OnConfirmClicked()
        {
            _view.Disable();
            _view.OnEdit -= OnEditClicked;
            _view.OnConfirm -= OnConfirmClicked;
            _context.EnableSagaPlugin();
            _machine.changeState<JoinRoomState>();
        }

        private void OnEditClicked()
        {
            _view.Disable();
            _view.OnEdit -= OnEditClicked;
            _view.OnConfirm -= OnConfirmClicked;
            _context.EnableSagaPlugin();
            _machine.changeState<EditUserDetailsState>();
        }

        public override void update(float deltaTime)
        {
            
        }
    }
}