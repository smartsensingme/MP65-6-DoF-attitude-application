function mp65_cube(port)
  % Application-specific MP65 viewer. The esp_stream_session_* calls below
  % provide generic serial framing/decoder bookkeeping; quaternion semantics,
  % validity, reference mode and rendering belong exclusively to this file.
  % No argument: text port picker. Optional port preserves scripted use.
  % Live commands use nonblocking kbhit; no input() during reception.
  if nargin == 0, port = esp_stream_select_port(); endif
  if isempty(port), return; endif
  live = []; fig = [];
  unwind_protect
    live = esp_stream_session_open(port);
    state = live.state;
    fig = figure('name', 'MP65 live attitude', 'keypressfcn', @cube_key);
    setappdata(fig, 'tilt', true); setappdata(fig, 'reference_pending', false);
    setappdata(fig, 'exit', false); setappdata(fig, 'status', false);
    setappdata(fig, 'menu', false);
    vertices = [-1 -1 -1;1 -1 -1;1 1 -1;-1 1 -1; ...
                -1 -1 1;1 -1 1;1 1 1;-1 1 1];
    faces = [1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8];
    cube = patch('vertices', vertices, 'faces', faces, ...
                 'facevertexcdata', (1:6)', 'facecolor', 'flat');
    axis([-2 2 -2 2 -2 2]); axis equal; grid on; view(3);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    reference = eye(3); old_tilt = true; latest = []; session_id = [];
    last_data = tic(); last_draw = tic(); started = tic();
    fprintf('\nConectado a %s. Mantenha o sensor parado para calibrar.\n', port);
    cube_menu();
    while ishandle(fig) && !getappdata(fig, 'exit')
      % At most one keyboard character per iteration; input never halts USB.
      key = kbhit(1);
      if !isempty(key), cube_command(fig, lower(key)); endif
      if getappdata(fig, 'exit'), break; endif
      % Session handles transport freshness; this application handles attitude
      % validity and quaternion interpretation below.
      [live,blocks] = esp_stream_session_poll(live);
      state = live.state;
      for k = 1:numel(blocks)
        b=blocks{k}; sample=mp65_attitude_sample(b);
        if !isempty(sample)
          if isempty(session_id) || session_id!=b.session
            reference=eye(3); session_id=b.session;
          endif
          latest=sample; last_data=tic();
        endif
      endfor
      if getappdata(fig, 'menu')
        cube_menu(); setappdata(fig, 'menu', false);
      endif
      if getappdata(fig, 'status')
        ready = !isempty(latest) && latest.ready;
        reception = esp_stream_session_stats(live);
        fprintf('\n%.1f s | blocos=%d | CRC=%d | frames perdidos=%d | ready=%d | stale=%d\n', ...
          toc(started), reception.blocks_received, reception.crc_errors, ...
          reception.lost_frames, ready, isempty(latest) || toc(last_data)>0.5);
        fprintf('Modo: %s\n', cube_mode(getappdata(fig, 'tilt')));
        setappdata(fig, 'status', false);
      endif
      tilt = getappdata(fig, 'tilt');
      if tilt != old_tilt, reference = eye(3); old_tilt = tilt; endif
      % Render at most 25 Hz, independently of transport polling.
      if toc(last_draw) >= 0.04
        if isempty(latest) || toc(last_data) > 0.5
          title('STALE / waiting for USB data');
        elseif !latest.ready
          title('Calibrating: keep sensor stationary');
        else
          R = attitude_rotation(latest.q, tilt);
          if getappdata(fig, 'reference_pending')
            reference = R; setappdata(fig, 'reference_pending', false);
            fprintf('Referência visual definida.\n');
          endif
          set(cube, 'vertices', (reference' * R * vertices')');
          title(sprintf('%s | gaps: %d', cube_mode(tilt), state.lost_frames));
        endif
        drawnow(); last_draw = tic();
      endif
      pause(0.01);
    endwhile
  unwind_protect_cleanup
    if !isempty(live), live = esp_stream_session_close(live); endif
    if !isempty(fig) && ishandle(fig), close(fig); endif
    fprintf('\nSessão encerrada; conexão serial liberada.\n');
  end_unwind_protect
endfunction

function mode = cube_mode(tilt)
  mode = '3D: yaw relativo (sujeito a deriva)';
  if tilt, mode = 'Inclinação (yaw oculto)'; endif
endfunction

function cube_menu()
  fprintf(['\nMenu ao vivo — pressione a tecla no terminal (sem Enter):\n', ...
    '  1) Somente inclinação\n  2) Orientação 3D completa\n', ...
    '  3) Definir referência visual\n  4) Mostrar diagnóstico da recepção\n', ...
    '  5) Mostrar este menu\n  0) Sair e liberar a porta\n', ...
    'Na figura: T alterna modo; Espaço define referência; Q sai.\n']);
endfunction

function cube_command(fig, key)
  switch key
    case '1', setappdata(fig, 'tilt', true);
    case '2', setappdata(fig, 'tilt', false);
    case {'3', ' ', 'space'}, setappdata(fig, 'reference_pending', true);
    case '4', setappdata(fig, 'status', true);
    case {'5', 'm'}, setappdata(fig, 'menu', true);
    case {'0', 'q', 'escape'}, setappdata(fig, 'exit', true);
    case 't', setappdata(fig, 'tilt', !getappdata(fig, 'tilt'));
  endswitch
endfunction

function cube_key(fig, event)
  cube_command(fig, event.Key);
endfunction
