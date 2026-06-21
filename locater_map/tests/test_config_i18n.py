from locater_map.config_loader import load_config, resolve_resource
from locater_map.i18n import normalize_language, tr


def test_default_map_assets_exist():
    config = load_config()
    background = resolve_resource(config, config["map"]["background_image"])
    labeled = resolve_resource(config, config["map"]["labeled_background_image"])
    prior = resolve_resource(config, config["map"]["prior_map_config"])
    assert background is not None and background.exists()
    assert labeled is not None and labeled.exists()
    assert prior is not None and prior.exists()
    assert config["map"]["field_width_cm"] == 1215.0
    assert config["map"]["field_height_cm"] == 1210.0


def test_i18n_default_chinese_and_english_toggle_text():
    assert normalize_language(None) == "zh"
    assert normalize_language("en") == "en"
    assert tr("zh", "language_toggle") == "English"
    assert tr("en", "language_toggle") == "中文"
    assert tr("zh", "sensor_encoder_1") == "正交编码轮1 (Encoder 1)"
    assert tr("zh", "sensor_encoder_2") == "正交编码轮2 (Encoder 2)"
    assert tr("en", "sensor_encoder_1") == "Encoder 1 (正交编码轮1)"
    assert tr("zh", "sensor_pulse_received") == "已收到脉冲"
