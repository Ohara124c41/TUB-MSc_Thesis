mainfilename=thesis

pdf:
	pdflatex ${mainfilename}
	bibtex ${mainfilename}||true
	pdflatex ${mainfilename}
	pdflatex ${mainfilename}

clean:
	rm -f ${mainfilename}.{ps,log,aux,out,dvi,bbl,blg}
