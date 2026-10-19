import { mountDetection } from '../ml_ui.js';
export default function register(registerTab){let dispose=null;
  registerTab({id:'objects',title:'Objects',order:2.6,render(el){dispose=mountDetection(el);},onLeave(){dispose?.();dispose=null;}});
}
