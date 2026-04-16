function DrawFinishLine(ax, x_start, road_width)
    % Disegna una striscia a scacchi sulla strada e un palo laterale
    
    sq_size = 1.0; % Dimensione del singolo scacco (1 metro)
    z_lift = 0.02; % Leggermente sopra l'asfalto per non sfarfallare
    
    % --- 1. STRISCIA SULL'ASFALTO (Doppia Fila) ---
    % Facciamo 2 file di scacchi per profondità
    for row = 0:1 
        % Spazzoliamo tutta la larghezza da -road_width a +road_width
        for y = -road_width : sq_size : (road_width - sq_size)
            
            % Logica Scacchiera: se (riga + colonna) è pari -> Nero, dispari -> Bianco
            col_idx = (y + road_width) / sq_size;
            is_black = mod(col_idx + row, 2) == 0;
            
            if is_black
                col = [0.1 0.1 0.1]; % Nero (non puro per vederlo meglio)
            else
                col = [0.9 0.9 0.9]; % Bianco
            end
            
            % Coordinate del quadrato
            X = [x_start + row*sq_size, x_start + (row+1)*sq_size, ...
                 x_start + (row+1)*sq_size, x_start + row*sq_size];
            Y = [y, y, y+sq_size, y+sq_size];
            Z = [z_lift, z_lift, z_lift, z_lift];
            
            patch(ax, X, Y, Z, col, 'EdgeColor', 'none');
        end
    end
    
    % --- 2. PALO E BANDIERA LATERALE (Opzionale, scenografico) ---
    pole_x = x_start;
    pole_y = road_width + 1; % 1 metro fuori strada
    pole_h = 5;
    
    % Disegna Palo (Linea spessa nera)
    %plot3(ax, [pole_x, pole_x], [pole_y, pole_y], [0, pole_h], 'k-', 'LineWidth', 4);
    
    % Disegna Bandiera a Scacchi sul palo (sventola verso X negativo)
    flag_size = 0.5;
    rows = 4; cols = 6;
    start_z = pole_h;
    
    for r = 0:rows-1
        for c = 0:cols-1
            % Alternanza colori
            if mod(r+c, 2) == 0, col='k'; else, col='w'; end
            
            % Coordinate scacco bandiera
            Xf = [pole_x - c*flag_size, pole_x - (c+1)*flag_size, ...
                  pole_x - (c+1)*flag_size, pole_x - c*flag_size];
            % Y leggermente variabile per simulare vento (sinusoidale)
            wave = sin(c*0.5)*0.2; 
            Yf = [pole_y + wave, pole_y + wave, pole_y + wave, pole_y + wave];
            Zf = [start_z - r*flag_size, start_z - r*flag_size, ...
                  start_z - (r+1)*flag_size, start_z - (r+1)*flag_size];
              
            patch(ax, Xf, Yf, Zf, col, 'EdgeColor', 'none');
        end
    end
end