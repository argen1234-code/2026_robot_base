"""Exercise pointer effects against the existing intercepted hardware fixtures."""
import json
from urllib.parse import unquote

from playwright.sync_api import expect


def check_effects(page, context, output, status, snap, submitted):
    checks = []
    page.emulate_media(reduced_motion='no-preference')
    page.click('#open-settings')
    page.click('#reset-settings')
    page.click('#close-settings')
    command_count = lambda: sum(action != 'probes' for action, payload in submitted)
    initial_commands = command_count()

    def sweep(selector, settle=True):
        surface = page.locator(selector)
        if settle:
            page.wait_for_timeout(650)
            surface.evaluate("node=>node.scrollIntoView({block:'center',behavior:'instant'})")
            page.wait_for_timeout(850)
        rect = surface.bounding_box()
        assert rect and rect['width'] > 0 and rect['height'] > 0, selector
        y = max(100, rect['y'] + rect['height'] * .5)
        # Put the first point outside the target so each surface gets a fresh entry.
        page.mouse.move(rect['x'] + 2, y)
        page.mouse.move(rect['x'] + rect['width'] * .8, y, steps=12)
        return surface

    page.click('#tab-control')
    for index in range(1, 5):
        surface = sweep(f'.metrics article:nth-child({index})', index == 1)
        expect(surface).to_have_class('signal-distorting', timeout=4000)
        assert page.locator('.signal-overlay').count() <= 6
    page.screenshot(path=str(output / '11-metrics-hover.png'))
    assert command_count() == initial_commands, 'pointer motion must not submit commands'
    checks.append('all four metric surfaces start on cross-card movement; overlays are bounded and pointer-transparent')
    assert page.locator('.signal-overlay').evaluate_all(
        "nodes=>nodes.every(node=>getComputedStyle(node).pointerEvents==='none')")

    snap['motor2']['velocity_rad_s'] = 7.25
    expect(page.locator('#velocity')).to_have_text('7.250', timeout=5000)
    sweep('.metrics article:first-child')
    page.wait_for_timeout(1400)
    expect(page.locator('.signal-overlay')).to_have_count(0)
    expect(page.locator('.signal-distorting')).to_have_count(0)
    checks.append('fixture telemetry updates through active effects and restores native content after decay')

    page.click('#tab-telemetry')
    page.evaluate('''() => {
      window.effectSnapshots=[];
      window.originalImageDecode=Image.prototype.decode;
      Image.prototype.decode=function(){
        if(this.src.startsWith('data:image/svg+xml'))window.effectSnapshots.push(this.src);
        return window.originalImageDecode.call(this);
      };
    }''')
    sweep('.chart-wrap')
    expect(page.locator('.chart-wrap')).to_have_class('chart-wrap signal-distorting', timeout=4000)
    snapshots = [unquote(value) for value in page.evaluate('effectSnapshots')]
    assert any('id="chart"' in value and 'data:image/png' in value and '<img' in value for value in snapshots), 'live curve canvas must be preserved in snapshots'
    page.screenshot(path=str(output / '12-chart-hover.png'))
    page.locator('#chart-metric').focus()
    expect(page.locator('.signal-overlay')).to_have_count(0)
    page.select_option('#chart-metric', 'temperature_c')
    assert command_count() == initial_commands
    checks.append('live chart pixels copied into snapshots; keyboard/select restore native controls without requests')

    page.click('#tab-diagnostics')
    sweep('.diagnostic-item:first-child')
    expect(page.locator('.diagnostic-item:first-child')).to_have_class('diagnostic-item signal-distorting', timeout=4000)
    page.click('#tab-led')
    sweep('.led-live > div:first-child')
    expect(page.locator('.led-live > div:first-child')).to_have_class('signal-distorting', timeout=4000)
    page.click('#tab-guide')
    sweep('.guide-grid article:first-child')
    expect(page.locator('.guide-grid article:first-child')).to_have_class('signal-distorting', timeout=4000)
    checks.append('diagnostics replaced by polling, LED live state and guide content support hover effects')

    footer = page.locator('#footerWordmark')
    footer.scroll_into_view_if_needed()
    page.wait_for_timeout(400)
    before = page.locator('#footerDistortion').evaluate('(node)=>node.toDataURL()')
    sweep('#footerWordmark', False)
    during = page.locator('#footerDistortion').evaluate('(node)=>node.toDataURL()')
    assert before != during, 'footer pixels should distort with pointer velocity'
    page.screenshot(path=str(output / '13-footer-hover.png'))
    page.wait_for_timeout(1600)
    assert page.locator('#footerDistortion').evaluate('(node)=>node.toDataURL()') == before
    checks.append('footer wordmark distorts and returns exactly to its static pixels')

    page.click('#open-settings')
    page.uncheck('#signalMotionEnabled')
    page.uncheck('#footerMotionEnabled')
    page.click('#close-settings')
    page.reload()
    page.wait_for_function('() => !!data && !busy')
    page.click('#open-settings')
    expect(page.locator('#signalMotionEnabled')).not_to_be_checked()
    expect(page.locator('#footerMotionEnabled')).not_to_be_checked()
    page.click('#reset-settings')
    expect(page.locator('#signalMotionEnabled')).to_be_checked()
    expect(page.locator('#footerMotionEnabled')).to_be_checked()
    page.click('#close-settings')
    checks.append('independent effect toggles persist and reset with the existing appearance settings')

    page.emulate_media(reduced_motion='reduce')
    sweep('.session-intro h2')
    expect(page.locator('.signal-overlay')).to_have_count(0)
    expect(page.locator('#footerWordmark')).to_have_class('footer-wordmark is-ready is-static')
    page.emulate_media(reduced_motion='no-preference')
    page.click('#tab-control')
    sweep('.metrics article:first-child')
    page.evaluate("window.dispatchEvent(new Event('blur'))")
    expect(page.locator('.signal-overlay')).to_have_count(0)
    page.locator('#target').fill('0.9')
    page.locator('#target').press('ArrowRight')
    expect(page.locator('#tab-control')).to_have_attribute('aria-selected', 'true')
    assert command_count() == initial_commands
    checks.append('reduced motion and blur clear overlays; numeric inputs retain editing without issuing commands')

    # Exercise both native fallback and asynchronous worker-load failure.
    for failure in ('unsupported', 'worker-load'):
        fallback = context.new_page()
        fallback_errors = []
        fallback.on('pageerror', lambda error: fallback_errors.append(str(error)))
        if failure == 'unsupported':
            fallback.add_init_script('HTMLCanvasElement.prototype.transferControlToOffscreen=undefined')
        else:
            fallback.add_init_script("window.Worker=class extends Worker{constructor(){super('/missing-pixel-worker.js')}}")
        fallback.goto(page.url)
        fallback.emulate_media(reduced_motion='no-preference')
        fallback.click('#open-settings')
        fallback.click('#reset-settings')
        fallback.click('#close-settings')
        fallback.wait_for_timeout(500)
        one = fallback.locator('#pixelField').evaluate('(node)=>node.toDataURL()')
        fallback.wait_for_timeout(150)
        two = fallback.locator('#pixelField').evaluate('(node)=>node.toDataURL()')
        assert one != two, failure
        fallback.click('#open-settings')
        fallback.uncheck('#motion-enabled')
        fallback.wait_for_timeout(150)
        fallback.click('#close-settings')
        fallback.evaluate('window.scrollTo(0,0)')
        # Main-thread fallback canvas remains readable and empty when disabled.
        assert fallback.locator('#pixelField').evaluate("node=>{const ctx=node.getContext('2d');return !ctx||!ctx.getImageData(0,0,node.width,node.height).data.some(v=>v)}")
        assert not fallback_errors, fallback_errors
        fallback.close()
    checks.append('hero point field runs with OffscreenCanvas support, native fallback and worker failure; disable clears pixels')

    report = {'passed': True, 'hardware_tested': False, 'template_commit': '71f6c076b7723165c5e875ad8a2ecdb82b27a908', 'checks': checks}
    (output / 'effects-regression.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    return checks
