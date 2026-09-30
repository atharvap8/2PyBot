/* tabs/safety.js — Safety configuration.
 * Owns firmware parameter group "SAFETY". Rows are generated from the
 * P? dump, so adding a parameter in firmware makes it appear here.
 */
export default function register(registerTab) {
  registerTab({
    id: 'safety', title: 'Safety', order: 50,
    render(el, ctx) {
      ctx.renderGroup(el, 'SAFETY', {
        title: 'Safety',
        hint: 'Arming window, fall cutoffs and link timeouts. Widening MAXTILT does not make the robot better at recovering; it just delays the cutoff.'.replace(/^'|'$/g, '')
      });
    }
  });
}
