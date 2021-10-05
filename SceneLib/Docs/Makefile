all: models.ps

distrib: all cleanexceptps

models.ps: models.dvi
	dvips models.dvi -o models.ps

models.dvi: models.tex bib.bib Makefile
	latex models.tex
	bibtex models
	latex models.tex
	latex models.tex

cleanexceptps: 
	rm -f *~ *.dvi *.aux *.log *.toc *.bbl *.blg

clean: cleanexceptps
	rm -f *.ps