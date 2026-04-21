let selectedColor = null;

document.querySelectorAll('.color-btn').forEach(btn => {
  btn.addEventListener('click', () => {
    document.querySelectorAll('.color-btn').forEach(b => b.classList.remove('selected'));
    btn.classList.add('selected');
    selectedColor = btn.querySelector('input').value;
    fetch('/set_color?color=' + selectedColor);
  });
});

function poll() {
  fetch('/frame')
    .then(r => r.json())
    .then(data => {
      if (data.img) {
        document.getElementById('feed').src = 'data:image/jpeg;base64,' + data.img;
        document.getElementById('status').textContent = 'Flux actif';
      }
      const det = document.getElementById('detection');
      if (data.detected) {
        det.className = 'detected';
        det.textContent = `✅ ${data.color.toUpperCase()} détecté — X: ${data.cx} / ${data.frame_width}`;
      } else {
        det.className = 'missing';
        det.textContent = selectedColor
          ? `❌ Cube ${selectedColor} non détecté — robot en recherche`
          : '⚠️ Aucune couleur sélectionnée';
      }
      setTimeout(poll, 80);
    })
    .catch(() => {
      document.getElementById('status').textContent = 'Connexion perdue...';
      setTimeout(poll, 500);
    });
}

poll();