/* tabs/led.js — LED Ring configuration.
 * Owns firmware parameter group "LED". Rows are generated from the
 * P? dump, so adding a parameter in firmware makes it appear here.
 */
export default function register(registerTab) {
  registerTab({
    id: 'led', title: 'LED Ring', order: 80,
    render(el, ctx) {
      ctx.renderGroup(el, 'LED', {
        title: 'LED Ring',
        hint: 'Brightness cap and which LED faces forward. 16 LEDs at full white draw ~1 A.'.replace(/^'|'$/g, '')
      });
    }
  });
}
