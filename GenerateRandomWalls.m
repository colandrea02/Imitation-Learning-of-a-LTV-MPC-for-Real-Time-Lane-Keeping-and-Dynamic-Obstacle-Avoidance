function Walls = GenerateRandomWalls(N_walls, x_start, x_end)
    % Divide lo spazio disponibile in "slot" uguali e posiziona un muro
    % in una zona sicura all'interno di ogni slot.
    % Garantisce esattamente N_walls con distanze min e max controllate.
    
    Walls = [];
    
    % Spazio totale disponibile per i muri
    total_space = x_end - x_start; 
    
    % Controllo di sicurezza: c'è abbastanza spazio per N muri?
    % Assumiamo che servano almeno 30m in media per ogni muro (inclusa la larghezza)
    if total_space < N_walls * 30
        warning('Spazio insufficiente per %d muri. Riduco il numero.', N_walls);
        N_walls = floor(total_space / 30);
        if N_walls == 0
            error('Spazio troppo piccolo per generare anche solo un muro!');
        end
    end
    
    % Calcoliamo la dimensione di uno slot
    slot_size = total_space / N_walls;
    
    % Definiamo un margine interno allo slot per garantire una distanza minima e massima
    % Es: slot = 75m. 
    % margin = 10m. Il muro nascerà a caso nei 55m centrali dello slot.
    % Questo garantisce che tra un muro (es. a fine slot 1) e il successivo (inizio slot 2)
    % ci siano ALMENO (margin + margin) = 20m di distanza.
    % E MASSIMO (slot_size - margin) + (slot_size - margin) circa.
    slot_margin = 15; % Distanza minima garantita tra muri = 30m (15+15)
    
    % Se lo slot è troppo piccolo per il margine, lo riduciamo
    if slot_size <= slot_margin * 2
        slot_margin = (slot_size * 0.2); 
    end
    
    for i = 1:N_walls
        % Inizio e fine dello slot di competenza per questo muro
        slot_start = x_start + (i - 1) * slot_size;
        slot_end   = x_start + i * slot_size;
        
        % Zona sicura dove può nascere (Jitter zone)
        spawn_min = slot_start + slot_margin;
        spawn_max = slot_end - slot_margin;
        
        % Generazione Posizione X
        Walls(i).x_pos = spawn_min + (spawn_max - spawn_min) * rand();
        
        % --- Parametri Geometrici ---
        Walls(i).width = 3 + (5-3) * rand();
        Walls(i).h_block = 3 + (5-3) * rand();
        Walls(i).y_close = -2 + (2 - (-2)) * rand();
        
        if rand > 0.5
            Walls(i).y_open = 3 + (5-3) * rand();
        else
            Walls(i).y_open = -5 + (-3 - (-5)) * rand();
        end
    end
end