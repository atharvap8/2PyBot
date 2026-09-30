/* tabs/stepper.js — Stepper configuration.
 * Owns firmware parameter group "STEPPER". Rows are generated from the
 * P? dump, so adding a parameter in firmware makes it appear here.
 */
export default function register(registerTab) {
  registerTab({
    id: 'stepper', title: 'Stepper', order: 20,
    render(el, ctx) {
      ctx.renderGroup(el, 'STEPPER', {
        title: 'Stepper',
        hint: 'Geometry and driver limits. WHEELR is the LOADED rolling radius: measure it (push 2 m, read encoder delta) rather than halving the moulded diameter. Everything derived updates live below.'.replace(/^'|'$/g, '')
      });
    }
  });
}
