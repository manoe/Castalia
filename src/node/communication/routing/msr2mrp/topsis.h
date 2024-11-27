//*******************************************************************************
//*  Copyright: Balint Aron Uveges, 2024                                        *
//*  Developed at Pazmany Peter Catholic University,                            *
//*               Faculty of Information Technology and Bionics                 *
//*  Author(s): Balint Aron Uveges                                              *
//*  This file is distributed under the terms in the attached LICENSE file.     *
//*                                                                             *
//*******************************************************************************


#ifndef _TOPSIS_H_
#define _TOPSIS_H_

#include <cmath>
#include <armadillo>

using namespace arma;

struct ps_alt {
    int sink;
    int pathid;
    double rank;
};

class TopsisEngine {
    private:
        mat table;
        int alt_num;
        std::vector<ps_alt> alts;
        colvec weights;
        std::vector<bool> attrs;
    protected:
        mat normalizeMatrix() {
            mat pow_table = square(table);
            mat norm_table = table;
            rowvec norm(table.n_cols,fill::zeros);
            for(int i=0; i < table.n_cols ; ++i) {
                norm(i)=sum(pow_table.col(i));
            }
            norm=sqrt(norm);
            for(int i=0; i < table.n_rows ; ++i) {
                norm_table.row(i) / norm;
            }
            return norm_table;
        };
        mat applyWeights() {
            mat norm_w_table = table;
            for(int i=0 ; i < table.n_cols ; ++i) {

            }
            return norm_w_table;
        };
        rowvec calculateIdealBest(mat table) {
            rowvec v_best;
            for(int i=0; i<table.n_cols; ++i) {
                if(attrs[i]) {
                   v_best[i]=max(table.col(i));
                } else {
                    v_best[i]=min(table.col(i));
                }
            }
            return v_best;
        };
        rowvec calculateIdealWorst(mat table) {
            rowvec v_worst;
            for(int i=0; i<table.n_cols; ++i) {
                if(attrs[i]) {
                    v_worst[i]=min(table.col(i));
                } else {
                    v_worst[i]=max(table.col(i));
                }
            }
            return v_worst;
        };
        colvec calculateSeparation(mat table, rowvec vec) {
            colvec c;
            for(int i=0; i < table.n_rows; ++i) {
                c[i]=sqrt(sum(square(table.row(i)-vec)));
            }
            return c;
        };
        colvec calculateCloseness(colvec id_pos_sep, colvec id_neg_sep) {
            return id_neg_sep / (id_neg_sep+id_pos_sep);
        };

    public:
        TopsisEngine(int alt_num): alt_num(alt_num),
                                   weights({1.0/4.0, 1.0/4.0, 1.0/4.0, 1.0/4.0}),
                                   attrs({false,true,true,true}) {
            table=mat(0, 4, fill::zeros);
        };
        void addAlternative(ps_alt id, int hop, double pdr, double nrg, double env) {
            alts.push_back(id);
            rowvec v({static_cast<double>(hop), pdr, nrg, env});
            table.row(table.n_rows)=v;
        };
        void addWeights(std::vector<double> w_arr) {
            weights=w_arr;
        };
        std::vector<ps_alt> getRanking() {
            std::vector<ps_alt> res;
            auto n_table=normalizeMatrix();
            auto n_w_table=applyWeights();
            auto v_best=calculateIdealBest(n_w_table);
            auto v_worst=calculateIdealWorst(n_w_table);
            auto id_pos_sep=calculateSeparation(n_w_table,v_best);
            auto id_neg_sep=calculateSeparation(n_w_table,v_worst);
            auto closeness=calculateCloseness(id_pos_sep,id_neg_sep);
            uvec index=sort_index(closeness,"descend");
            for(auto it=index.begin() ; it != index.end() ; ++it) {
                ps_alt e=alts[*it];
                e.rank=closeness[*it];
                res.push_back(e);
            }
            return res;
        };
};


#endif // _TOPSIS_H_
